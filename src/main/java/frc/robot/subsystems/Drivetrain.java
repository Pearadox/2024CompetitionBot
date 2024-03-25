// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.SmarterDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;

public class Drivetrain extends SubsystemBase {
  private SwerveModule leftFront;
  private SwerveModule rightFront;
  private SwerveModule leftBack;
  private SwerveModule rightBack;

  private SlewRateLimiter frontLimiter;
  private SlewRateLimiter sideLimiter;
  private SlewRateLimiter turnLimiter;

  private PIDController alignPIDController;

  private Pigeon2 gyro;

  private static final NetworkTable llTable = NetworkTableInstance.getDefault().getTable(VisionConstants.SHOOTER_LL_NAME);

  public enum DriveMode{
    Normal, Align
  }

  private DriveMode driveMode = DriveMode.Normal;

  public static final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
  private GenericEntry leftFrontStateEntry;
  private GenericEntry rightFrontStateEntry;
  private GenericEntry leftBackStateEntry;
  private GenericEntry rightBackStateEntry;
  private GenericEntry robotAngleEntry;
  private GenericEntry angularSpeedEntry;

  private static final Drivetrain DRIVETRAIN = new Drivetrain();

  public static Drivetrain getInstance(){
    return DRIVETRAIN;
  }

  /** Creates a new SwerveDrivetrain. */
  public Drivetrain() {
    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      }
      catch(Exception e){}
    }).start();

    leftFront = new SwerveModule(
      SwerveConstants.LEFT_FRONT_DRIVE_ID, 
      SwerveConstants.LEFT_FRONT_TURN_ID, 
      false, 
      true, 
      SwerveConstants.LEFT_FRONT_CANCODER_ID, 
      SwerveConstants.LEFT_FRONT_OFFSET);

    rightFront = new SwerveModule(
      SwerveConstants.RIGHT_FRONT_DRIVE_ID, 
      SwerveConstants.RIGHT_FRONT_TURN_ID, 
      false, 
      true, 
      SwerveConstants.RIGHT_FRONT_CANCODER_ID, 
      SwerveConstants.RIGHT_FRONT_OFFSET);

    leftBack = new SwerveModule(
      SwerveConstants.LEFT_BACK_DRIVE_ID, 
      SwerveConstants.LEFT_BACK_TURN_ID, 
      false, 
      true, 
      SwerveConstants.LEFT_BACK_CANCODER_ID, 
      SwerveConstants.LEFT_BACK_OFFSET);
    
    rightBack = new SwerveModule(
      SwerveConstants.RIGHT_BACK_DRIVE_ID, 
      SwerveConstants.RIGHT_BACK_TURN_ID, 
      false, 
      true, 
      SwerveConstants.RIGHT_BACK_CANCODER_ID, 
      SwerveConstants.RIGHT_BACK_OFFSET);

    frontLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
    sideLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
    turnLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);

    alignPIDController = new PIDController(SwerveConstants.kP_PERCENT, 0, 0);

    gyro = new Pigeon2(SwerveConstants.PIGEON_ID);
    
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetPose,
      this::getRobotRelativeSpeeds,
      this::driveRobotRelative,
      SwerveConstants.AUTO_CONFIG,
      () -> isRedAlliance(),
      this);

    leftFrontStateEntry = swerveTab.add("Left Front Module State", leftFront.getState().toString()).withSize(4, 1).withPosition(0, 0).getEntry();
    rightFrontStateEntry = swerveTab.add("Right Front Module State", rightFront.getState().toString()).withSize(4, 1).withPosition(0, 1).getEntry();
    leftBackStateEntry = swerveTab.add("Left Back Module State", leftBack.getState().toString()).withSize(4, 1).withPosition(0, 2).getEntry();
    rightBackStateEntry = swerveTab.add("Right Back Module State", rightBack.getState().toString()).withSize(4, 1).withPosition(0, 3).getEntry();
    robotAngleEntry = swerveTab.add("Robot Angle", getHeading()).withSize(1, 1).withPosition(4, 1).getEntry();
    angularSpeedEntry = swerveTab.add("Angular Speed", new DecimalFormat("#.00").format((-gyro.getRate() / 180)) + "\u03C0" + " rad/s").withSize(1, 1).withPosition(5, 1).getEntry();
  }

  @Override
  public void periodic() {
    RobotContainer.poseEstimation.updateOdometry(getHeadingRotation2d(), getModulePositions());

    SmarterDashboard.putString("Drive Mode", getDriveMode().toString(), "Drivetrain");
    SmarterDashboard.putString("Left Front Module State", leftFront.getState().toString(), "Drivetrain");
    SmarterDashboard.putString("Right Front Module State", rightFront.getState().toString(), "Drivetrain");
    SmarterDashboard.putString("Left Back Module State", leftBack.getState().toString(), "Drivetrain");
    SmarterDashboard.putString("Right Back Module State", rightBack.getState().toString(), "Drivetrain");
    SmarterDashboard.putNumber("Robot Angle", getHeading(), "Drivetrain");
    SmarterDashboard.putString("Angular Speed", new DecimalFormat("#.00").format((-gyro.getRate() / 180)) + "\u03C0" + " rad/s", "Drivetrain");
    SmarterDashboard.putBoolean("Ready To Shoot", readyToShoot(), "Drivetrain");
    SmartDashboard.putString("Odometry", getPose().toString());
    Logger.recordOutput("Drivetrain/Odometry", getPose());

    leftFrontStateEntry.setString(leftFront.getState().toString());
    rightFrontStateEntry.setString(rightFront.getState().toString());
    leftBackStateEntry.setString(leftBack.getState().toString());
    rightBackStateEntry.setString(rightBack.getState().toString());
    robotAngleEntry.setDouble(getHeading());
    angularSpeedEntry.setString(new DecimalFormat("#.00").format((-gyro.getRate() / 180)) + "\u03C0" + "rad/s");
  }

  public void swerveDrive(double frontSpeed, double sideSpeed, double turnSpeed, 
    boolean fieldOriented, Translation2d centerOfRotation, boolean deadband){ //Drive with rotational speed control w/ joystick
    if(driveMode == DriveMode.Align && deadband){
      frontSpeed = Math.abs(frontSpeed) > 0.1 ? frontSpeed : 0;
      sideSpeed = Math.abs(sideSpeed) > 0.1 ? sideSpeed : 0;
    }
    else{
      frontSpeed = Math.abs(frontSpeed) > 0.1 ? frontSpeed : 0;
      sideSpeed = Math.abs(sideSpeed) > 0.1 ? sideSpeed : 0;
      turnSpeed = Math.abs(turnSpeed) > 0.1 ? turnSpeed : 0;
    }

    frontSpeed = frontLimiter.calculate(frontSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    sideSpeed = sideLimiter.calculate(sideSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    turnSpeed = turnLimiter.calculate(turnSpeed) * SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds;
    if(fieldOriented){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(frontSpeed, sideSpeed, turnSpeed, getHeadingRotation2d());
    }
    else{
      chassisSpeeds = new ChassisSpeeds(frontSpeed, sideSpeed, turnSpeed);
    }

    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

    setModuleStates(moduleStates);
  }

  public void swerveDrive(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation){ //Drive with field relative chassis speeds
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getHeadingRotation2d());

    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

    setModuleStates(moduleStates);
  }

  public void turnToHeading(double heading, Translation2d centerOfRotation){
    double turnSpeed;

    if(DriverStation.isAutonomousEnabled()){
      heading = isRedAlliance() ? -heading : heading;
    }

    double error = heading - getHeading();

    if(error > 180) {
      error -= 360;
    }
    else if(error < -180){
      error += 360;
    }
    
    if(Math.abs(error) > 1){
      turnSpeed = Math.signum(error) * SwerveConstants.kS_PERCENT + SwerveConstants.kP_PERCENT * error;
    }
    else{
      turnSpeed = 0;
    }

    turnSpeed = turnLimiter.calculate(turnSpeed) * SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turnSpeed, getHeadingRotation2d());

    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

    setModuleStates(moduleStates);
  }

  public boolean hasTurnedToHeading(double heading){
    heading = isRedAlliance() ? -heading : heading;
    return Math.abs(heading - getHeading()) < 1;
  }

  public void setAllIdleMode(boolean brake){
    if(brake){
      leftFront.setBrake(true);
      rightFront.setBrake(true);
      leftBack.setBrake(true);
      rightBack.setBrake(true);
    }
    else{
      leftFront.setBrake(false);
      rightFront.setBrake(false);
      leftBack.setBrake(false);
      rightBack.setBrake(false);
    }
  }

  public void resetAllEncoders(){
    leftFront.resetEncoders();
    rightFront.resetEncoders();
    leftBack.resetEncoders();
    rightBack.resetEncoders();
  }

  public Pose2d getPose(){
    return RobotContainer.poseEstimation.getEstimatedPose();
    
  }

  public void resetPose(Pose2d pose) {
    RobotContainer.poseEstimation.resetPose(pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }

  public ChassisSpeeds getFieldRelativeSpeeds(){
    ChassisSpeeds chassisSpeeds = SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    return ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, getHeadingRotation2d());
  }

  public void zeroHeading(){
    gyro.setYaw(0);
  }

  public void setHeading(double heading){
    gyro.setYaw(heading);
  }

  public double getHeading(){
    return Math.IEEEremainder(-gyro.getAngle(), 360); //clamp heading between -180 and 180
  }

  public Rotation2d getHeadingRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules(){
    leftFront.stop();
    leftBack.stop();
    rightFront.stop();
    rightBack.stop();
  }

  public void setModuleStates(SwerveModuleState[] moduleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.DRIVETRAIN_MAX_SPEED);
    leftFront.setDesiredState(moduleStates[0]);
    rightFront.setDesiredState(moduleStates[1]);
    leftBack.setDesiredState(moduleStates[2]);
    rightBack.setDesiredState(moduleStates[3]);
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = leftFront.getState();
    states[1] = rightFront.getState();
    states[2] = leftBack.getState();
    states[3] = rightBack.getState();
    return states;
  } 

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = leftFront.getPosition();
    positions[1] = rightFront.getPosition();
    positions[2] = leftBack.getPosition();
    positions[3] = rightBack.getPosition();
    return positions;
  }

  public boolean isRedAlliance(){
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  public double getAlignSpeed(){
    double alignSpeed;

    if(isRedAlliance()){
      if(llTable.getEntry("tid").getDouble(0) == 4){
        double[] camerapose_targetspace = llTable.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        double x = Math.abs(camerapose_targetspace[0]);
        double z = Math.abs(camerapose_targetspace[2]);
        double offset = Math.atan(x / z);

        double error = llTable.getEntry("tx").getDouble(0) + offset + 2.5;
        
        alignSpeed = Math.abs(error) > 0.9 ? -alignPIDController.calculate(llTable.getEntry("tx").getDouble(0) + offset, -2.5) : 0;
      }
      else{
        double alignAngle = getAlignAngle(4);

        double error = alignAngle - getHeading();

        if(error > 180) {
          error -= 360;
        }
        else if(error < -180){
          error += 360;
        }
        
        if(Math.abs(error) > 1){
          alignSpeed = Math.signum(-error) * SwerveConstants.kS_PERCENT + SwerveConstants.kP_PERCENT * -error;
        }
        else{
          alignSpeed = 0;
        }
      }
    }
    else{
      if(llTable.getEntry("tid").getDouble(0) == 7){
        double[] camerapose_targetspace = llTable.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        double x = Math.abs(camerapose_targetspace[0]);
        double z = Math.abs(camerapose_targetspace[2]);
        double offset = Math.atan(x / z);

        double error = llTable.getEntry("tx").getDouble(0) + offset + 2.5;
        
        alignSpeed = Math.abs(error) > 0.9 ? -alignPIDController.calculate(llTable.getEntry("tx").getDouble(0) + offset, -2.5) : 0;
      }
      else{
        double alignAngle = getAlignAngle(7);

        double error = alignAngle - getHeading();

        if(error > 180) {
          error -= 360;
        }
        else if(error < -180){
          error += 360;
        }
        
        if(Math.abs(error) > 1){
          alignSpeed = Math.signum(-error) * SwerveConstants.kS_PERCENT + SwerveConstants.kP_PERCENT * -error;
        }
        else{
          alignSpeed = 0;
        }
      }
    }

    return alignSpeed;
  }

  public double getAlignSpeedSourceAuto(){ 
    double alignSpeed;

    if(isRedAlliance()){
      if(llTable.getEntry("tid").getDouble(0) == 4){
        double[] camerapose_targetspace = llTable.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        double x = Math.abs(camerapose_targetspace[0]);
        double z = Math.abs(camerapose_targetspace[2]);
        double offset = Math.atan(x / z);

        double error = llTable.getEntry("tx").getDouble(0) + offset;
        
        alignSpeed = Math.abs(error) > 0.9 ? Math.signum(error) * SwerveConstants.kS_PERCENT + SwerveConstants.kP_PERCENT * error : 0;
      }
      else{
        double alignAngle = getAlignAngle(4);

        double error = alignAngle - getHeading();

        if(error > 180) {
          error -= 360;
        }
        else if(error < -180){
          error += 360;
        }
        
        if(Math.abs(error) > 1){
          alignSpeed = Math.signum(-error) * SwerveConstants.kS_PERCENT + SwerveConstants.kP_PERCENT * -error;
        }
        else{
          alignSpeed = 0;
        }
      }
    }
    else{
      if(llTable.getEntry("tid").getDouble(0) == 7){
        double[] camerapose_targetspace = llTable.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        double x = Math.abs(camerapose_targetspace[0]);
        double z = Math.abs(camerapose_targetspace[2]);
        double offset = Math.atan(x / z);

        double error = llTable.getEntry("tx").getDouble(0) + offset;
        
        alignSpeed = Math.abs(error) > 0.9 ? Math.signum(error) * SwerveConstants.kS_PERCENT + SwerveConstants.kP_PERCENT * error : 0;
      }
      else{
        double alignAngle = getAlignAngle(7);

        double error = alignAngle - getHeading();

        if(error > 180) {
          error -= 360;
        }
        else if(error < -180){
          error += 360;
        }
        
        if(Math.abs(error) > 1){
          alignSpeed = Math.signum(-error) * SwerveConstants.kS_PERCENT + SwerveConstants.kP_PERCENT * -error;
        }
        else{
          alignSpeed = 0;
        }
      }
    }

    return alignSpeed;
  }

  public boolean readyToShoot(){
    double[] camerapose_targetspace = llTable.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    double x = Math.abs(camerapose_targetspace[0]);
    double z = Math.abs(camerapose_targetspace[2]);
    double offset = Math.atan(x / z);

    double error = llTable.getEntry("tx").getDouble(0) + offset + 3.5;
    if(isRedAlliance()){
      return Math.abs(error) <= 0.9 && llTable.getEntry("tid").getDouble(0) == 4; 
    }
    else{
      return Math.abs(error) <= 0.9 && llTable.getEntry("tid").getDouble(0) == 7;
    }
  }

  public double getAlignAngle(int tagID){
    Pose2d tagPose = RobotContainer.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d();
    Pose2d robotPose = getPose();

    double deltaX = tagPose.getX() - robotPose.getX();
    double deltaY = tagPose.getY() - robotPose.getY() + Units.inchesToMeters(22.5);

    double alignAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

    if(!isRedAlliance()){
      alignAngle += 180;
      if(alignAngle > 180){
        alignAngle -= 360;
      }
    }

    return alignAngle;
  }

  public DriveMode getDriveMode(){
    return driveMode;
  }

  public void setNormalMode(){
    driveMode = DriveMode.Normal;
  }

  public void setAlignMode(){
    driveMode = DriveMode.Align;
  }

  public Command rumbleController(){
    return new InstantCommand(() -> RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0.25))
      .andThen(new WaitCommand(0.5))
      .andThen(new InstantCommand(() -> RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0)));
  }
}