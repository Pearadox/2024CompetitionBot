// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class ShootOnTheMove extends Command {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private Transport transport = Transport.getInstance();
  private Shooter shooter = Shooter.getInstance();
  private ChassisSpeeds chassisSpeeds;

  private SlewRateLimiter turnLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);

  private final NetworkTable llTable = NetworkTableInstance.getDefault().getTable(VisionConstants.SHOOTER_LL_NAME);

  private double timestamp;

  /** Creates a new ShootOnTheMove. */
  public ShootOnTheMove() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, transport, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timestamp = Timer.getFPGATimestamp();
    transport.setHolding(false);

    chassisSpeeds = drivetrain.getFieldRelativeSpeeds();
    if(Math.abs(chassisSpeeds.vxMetersPerSecond) >= Math.abs(chassisSpeeds.vyMetersPerSecond) && Math.abs(chassisSpeeds.vxMetersPerSecond) > 1.5){
      chassisSpeeds.vxMetersPerSecond = Math.signum(chassisSpeeds.vxMetersPerSecond) * 1.5;
      chassisSpeeds.vyMetersPerSecond *= Math.abs(chassisSpeeds.vxMetersPerSecond) / 1.5;
    }
    else if(Math.abs(chassisSpeeds.vyMetersPerSecond) >= Math.abs(chassisSpeeds.vxMetersPerSecond) && Math.abs(chassisSpeeds.vyMetersPerSecond) > 1.5){
      chassisSpeeds.vyMetersPerSecond = Math.signum(chassisSpeeds.vyMetersPerSecond) * 1.5;
      chassisSpeeds.vxMetersPerSecond *= Math.abs(chassisSpeeds.vyMetersPerSecond) / 1.5;
    }
    chassisSpeeds.omegaRadiansPerSecond = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnSpeed = turnLimiter.calculate(-drivetrain.getAlignSpeed()) * SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;
    chassisSpeeds.omegaRadiansPerSecond = turnSpeed;
    drivetrain.swerveDrive(chassisSpeeds, new Translation2d());

    if(shooter.hasPriorityTarget()){
      double[] botpose_targetspace = llTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
      double x = botpose_targetspace[0];
      double z = botpose_targetspace[2];
      double robotAngle_i = Math.atan(x / z);
      SmartDashboard.putNumber("Robot Angle initial", robotAngle_i);

      double v_n = shooter.getNoteVelocity() * 1.8;
      double v_n_pivot = v_n * Math.cos(robotAngle_i);
      SmartDashboard.putNumber("Note Velocity for pivot", v_n_pivot);
      double v_x = chassisSpeeds.vxMetersPerSecond;
      SmartDashboard.putNumber("ChassisSpeeds x", v_x);
      double deltaPivotAngle = Math.toDegrees(Math.asin(Math.abs(v_x) * Math.sin(Math.toRadians(shooter.calculatePivotAngle())) / Math.abs(v_n_pivot)));
      if(v_x > 0){
        deltaPivotAngle *= shooter.calculatePivotAngle() * 0.7 / 53.0;
      }
      else if(v_x < 0){
        deltaPivotAngle *= 53.0 * 0.7 / shooter.calculatePivotAngle();
      }
      SmartDashboard.putNumber("Delta Pivot Angle", deltaPivotAngle);
      double pivotAngle = v_x > 0 ? shooter.calculatePivotAngle() - deltaPivotAngle : shooter.calculatePivotAngle() + deltaPivotAngle;
      SmartDashboard.putNumber("Corrected Pivot Angle", pivotAngle);
      shooter.setPivotAngle(pivotAngle);
      shooter.setPivotPosition();
    }

    if(Timer.getFPGATimestamp() - timestamp > 0.7){
      transport.transportShoot();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transport.setHolding(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
