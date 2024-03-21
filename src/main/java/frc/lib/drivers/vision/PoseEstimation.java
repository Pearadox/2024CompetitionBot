
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.drivers.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;

public class PoseEstimation {
    private final SwerveDrivePoseEstimator poseEstimator;

    private final LimelightBackend[] backends;
    private final boolean[] backendToggles;

    private final TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(2);

    private static final double DIFFERENTIATION_TIME = Robot.defaultPeriodSecs;

    private Drivetrain drivetrain = Drivetrain.getInstance();

    private GenericEntry robotPoseEntry = Drivetrain.swerveTab.add("Robot Pose", new Pose2d().toString()).withSize(4, 1).withPosition(4, 0).getEntry();

    public PoseEstimation() {
        poseEstimator = new SwerveDrivePoseEstimator(
            SwerveConstants.DRIVE_KINEMATICS,
            drivetrain.getHeadingRotation2d(),
            drivetrain.getModulePositions(),
            new Pose2d(),
            SwerveConstants.ODOMETRY_STD_DEV,
            VecBuilder.fill(0, 0, 0) // will be overwritten for each measurement
        );

        backends = new LimelightBackend[2];
        backendToggles = new boolean[2];

        backends[0] = new LimelightBackend(VisionConstants.SHOOTER_LL_NAME);
        backends[1] = new LimelightBackend(VisionConstants.INTAKE_LL_NAME);
        backendToggles[0] = true;
        backendToggles[1] = true;
    }

    public void periodic() {
        for (int i = 0; i < backends.length; i++) {
            if (backendToggles[i]) {
                // this is a hack to get around an issue in `SwerveDrivePoseEstimator`
                // where two measurements cannot share the same timestamp
                double timestampOffset = 1e-9 * i;
                
                if(backends[i].isValid()){
                    backends[i].getMeasurement().map((measurement) -> {
                        measurement.timestamp += timestampOffset;
                        return measurement;
                    }).ifPresent(this::addVisionMeasurement);
                    
                    backends[i].getMeasurement().map((measurement) -> {
                        measurement.timestamp += timestampOffset;
                        return measurement;
                    }).ifPresent(this::loggingPose);
                }
            }
        }

        robotPoseEntry.setString(getEstimatedPose().toString());
        poseHistory.addSample(Timer.getFPGATimestamp(), poseEstimator.getEstimatedPosition());
    }

    public void updateOdometry(Rotation2d gyro, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(gyro, modulePositions);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Translation2d getEstimatedVelocity() {
        double now = Timer.getFPGATimestamp();

        Translation2d current = poseHistory.getSample(now).orElseGet(Pose2d::new).getTranslation();
        Translation2d previous = poseHistory.getSample(now - DIFFERENTIATION_TIME).orElseGet(Pose2d::new).getTranslation();

        return current.minus(previous).div(DIFFERENTIATION_TIME);
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(drivetrain.getHeadingRotation2d(), drivetrain.getModulePositions(), pose);
    }

    private void addVisionMeasurement(VisionBackend.Measurement measurement) {
        poseEstimator.addVisionMeasurement(measurement.pose.toPose2d(), measurement.timestamp, measurement.stdDeviation);
    }

    private void loggingPose(VisionBackend.Measurement measurement) {
        SmartDashboard.putString("Vision Pose", measurement.pose.toPose2d().toString());
        Logger.recordOutput("Drivetrain/Vision Pose", measurement.pose.toPose2d());
    }
}