// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkFlex;
import frc.lib.drivers.PearadoxSparkMax;
import frc.lib.util.LerpTable;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;

public class Shooter extends SubsystemBase {
  private PearadoxSparkFlex leftShooter;
  private PearadoxSparkMax rightShooter;
  
  private PearadoxSparkMax pivot;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder pivotEncoder;

  private SparkPIDController leftController;
  private SparkPIDController rightController;
  private SparkPIDController pivotController;

  private boolean zeroing = false;

  private static final NetworkTable llTable = NetworkTableInstance.getDefault().getTable(VisionConstants.LL_NAME);

  private double pivotPosition;
  private double[] botpose_targetspace = new double[6];
  public static final Drivetrain drivetrain = Drivetrain.getInstance();

  private LerpTable pivotLerp = new LerpTable();
  private LerpTable shooterLerp = new LerpTable();

  private static final Shooter SHOOTER = new Shooter();

  public static Shooter getInstance(){
    return SHOOTER;
  }

  public enum ShooterMode{
    Auto, Manual, Passing, Speaker
  }

  private ShooterMode shooterMode = ShooterMode.Auto;

  /** Creates a new Shooter. */
  public Shooter() {
    leftShooter = new PearadoxSparkFlex(ShooterConstants.LEFT_SHOOTER_ID, MotorType.kBrushless, IdleMode.kBrake, 50, false,
      ShooterConstants.LEFT_SHOOTER_kP, ShooterConstants.LEFT_SHOOTER_kI, ShooterConstants.LEFT_SHOOTER_kD,
      ShooterConstants.SHOOTER_MIN_OUTPUT, ShooterConstants.SHOOTER_MAX_OUTPUT);

    rightShooter = new PearadoxSparkMax(ShooterConstants.RIGHT_SHOOTER_ID, MotorType.kBrushless, IdleMode.kBrake, 50, true,
      ShooterConstants.RIGHT_SHOOTER_kP, ShooterConstants.RIGHT_SHOOTER_kI, ShooterConstants.RIGHT_SHOOTER_kD,
      ShooterConstants.SHOOTER_MIN_OUTPUT, ShooterConstants.SHOOTER_MAX_OUTPUT);

    pivot = new PearadoxSparkMax(ShooterConstants.PIVOT_ID, MotorType.kBrushless, IdleMode.kBrake, 50, false,
      ShooterConstants.PIVOT_kP, ShooterConstants.PIVOT_kI, ShooterConstants.PIVOT_kD,
      ShooterConstants.PIVOT_MIN_OUTPUT, ShooterConstants.PIVOT_MAX_OUTPUT);

    leftEncoder = leftShooter.getEncoder();
    pivotEncoder = pivot.getEncoder();
    
    leftController = leftShooter.getPIDController();
    rightController = rightShooter.getPIDController();
    pivotController = pivot.getPIDController();

    pivotLerp.addPoint(53, 19.7);
    pivotLerp.addPoint(50, 17.9);
    pivotLerp.addPoint(47, 16.4);
    pivotLerp.addPoint(44, 14.8);
    pivotLerp.addPoint(41, 13.5);
    pivotLerp.addPoint(38, 12.3);
    pivotLerp.addPoint(35, 11.3);
    pivotLerp.addPoint(32, 10.1);
    pivotLerp.addPoint(29, 9.4);
    pivotLerp.addPoint(26, 8.1);
    pivotLerp.addPoint(23, 7.2);
    pivotLerp.addPoint(20, 6.5);
    pivotLerp.addPoint(17, 6.1);
    pivotLerp.addPoint(14, 5.1);

    shooterLerp.addPoint(53, 7);
    shooterLerp.addPoint(47, 7);
    shooterLerp.addPoint(41, 7);
    shooterLerp.addPoint(35, 7.5);
    shooterLerp.addPoint(29, 8);
    shooterLerp.addPoint(23, 8.5);
    shooterLerp.addPoint(17, 9);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Left Speed", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Pivot Position", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Pivot Intended Position", pivotPosition);
    SmartDashboard.putNumber("Shooter Pivot Current", pivot.getOutputCurrent());
    SmartDashboard.putNumber("Shooter Pivot Intended Angle", calculatePivotAngle());  
    SmartDashboard.putBoolean("Shooter Has Priority Target", hasPriorityTarget());  
    SmartDashboard.putBoolean("AutoShoot Mode", shooterMode == ShooterMode.Auto ? true: false);
  }

  public void shooterHold(){
    double shooterVoltage = shooterLerp.interpolate(calculatePivotAngle());
    SmartDashboard.putNumber("Shooter Voltage", shooterVoltage);

    if(DriverStation.isAutonomousEnabled()){
      leftShooter.set(0.7);
      rightShooter.set(0.6);
    }
    else if(RobotContainer.driverController.getRawButton(XboxController.Button.kA.value)){
      leftController.setReference(
        3.35,
        ControlType.kVoltage,
        0);

      rightController.setReference(
        3.35,
        ControlType.kVoltage,
        0);
    }
    else if(shooterMode == ShooterMode.Passing){
      leftController.setReference(
        5.6,
        ControlType.kVoltage,
        0);

      rightController.setReference(
        3.6,
        ControlType.kVoltage,
        0);
    }
    else{
      leftController.setReference(
        shooterVoltage,
        ControlType.kVoltage,
        0);

      rightController.setReference(
        shooterVoltage - 2,
        ControlType.kVoltage,
        0);
    }
  }

  public void setShooterAuto(double speed){
    leftShooter.set(speed);
    rightShooter.set(speed);
  }

  public void pivotHold(){
    if(zeroing){
      pivot.set(-0.15);
    }
    else if(RobotContainer.driverController.getRawButton(XboxController.Button.kA.value)){
      pivotController.setReference(
        ShooterConstants.AMP_PIVOT_POSITION,
        ControlType.kPosition,
        0);
    }
    else if(shooterMode == ShooterMode.Passing){
      pivotController.setReference(
        ShooterConstants.PASSING_PIVOT_POSITION,
        ControlType.kPosition,
        0);
    }
    else if(shooterMode == ShooterMode.Speaker){
      pivotController.setReference(
        ShooterConstants.SPEAKER_PIVOT_POSITION,
        ControlType.kPosition,
        0);
    }
    else{
      pivotController.setReference(
        pivotPosition,
        ControlType.kPosition,
        0);
      
      if(shooterMode == ShooterMode.Auto && hasPriorityTarget()){
        setPivotAngle(calculatePivotAngle());
      }
    }

    if(RobotContainer.opController.getPOV() == 0){
      pivotPosition += 0.1;
    }
    else if(RobotContainer.opController.getPOV() == 180){
      pivotPosition -= 0.1;
    }
  }

  public void setZeroing(boolean zeroing){
    this.zeroing = zeroing;
  }

  public void resetPivotEncoder(){
    pivotEncoder.setPosition(0);
  }

  public void setBrakeMode(boolean brake){
    if(brake){
      leftShooter.setIdleMode(IdleMode.kBrake);
      rightShooter.setIdleMode(IdleMode.kBrake);
      pivot.setIdleMode(IdleMode.kBrake);

    }
    else{
      leftShooter.setIdleMode(IdleMode.kCoast);
      rightShooter.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setPivot(double speed){
    pivot.set(speed);
  }

  public double getPivotCurrent(){
    return pivot.getOutputCurrent();
  }

  public double calculatePivotAngle(){
    if(hasPriorityTarget()){
      botpose_targetspace = llTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    }

    double x = Math.abs(botpose_targetspace[0]);
    double z = Math.abs(botpose_targetspace[2]);
    double hypot = Math.hypot(x, z);

    double angle = Math.atan((FieldConstants.SPEAKER_HEIGHT - ShooterConstants.FLOOR_TO_SHOOTER) / hypot);
    return Units.radiansToDegrees(angle);
  }

  public void setPivotAngle(double angle){
    if(hasPriorityTarget()){
      pivotPosition = pivotLerp.interpolate(angle);
    }
  }

  public boolean hasPriorityTarget(){
    if(isRedAlliance()){
      return llTable.getEntry("tid").getDouble(0) == 4;
    }
    else{
      return llTable.getEntry("tid").getDouble(0) == 7;
    }
  }

  public void setPipeline(int index){
    llTable.getEntry("pipeline").setNumber(index);
  }

  public void setPivotPosition(double position){
    pivotPosition = position;
  }

  public ShooterMode getShooterMode(){
    return shooterMode;
  }

  public void setAutoMode(){
    shooterMode = ShooterMode.Auto;
  }

  public void setManualMode(){
    shooterMode = ShooterMode.Manual;
  }

  public void setPassingMode(){
    shooterMode = ShooterMode.Passing;
  }

  public void setSpeakerMode(){
    shooterMode = ShooterMode.Speaker;
  }

  public boolean isRedAlliance(){
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }
}
