// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
  private double pivotAdjust = 0;
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

  public static ShuffleboardTab driverTab;
  private GenericEntry leftShooterSpeedEntry;
  private GenericEntry rightShooterSpeedEntry;
  private GenericEntry shooterModeEntry;
  private GenericEntry pivotAdjustEntry;

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

    pivotLerp.addPoint(53, 20.3);
    pivotLerp.addPoint(50, 19.4);
    pivotLerp.addPoint(47, 17.8);
    pivotLerp.addPoint(44, 15.8);
    pivotLerp.addPoint(41, 14.8);
    pivotLerp.addPoint(38, 13.7);
    pivotLerp.addPoint(35, 13.0);
    pivotLerp.addPoint(32, 12.6);
    pivotLerp.addPoint(29, 11.4);
    pivotLerp.addPoint(26, 10.1);
    pivotLerp.addPoint(23, 8.9);
    pivotLerp.addPoint(20, 8.6);
    pivotLerp.addPoint(17, 8.0);
    pivotLerp.addPoint(14, 7.5);

    shooterLerp.addPoint(53, 6);
    shooterLerp.addPoint(47, 6.5);
    shooterLerp.addPoint(41, 7);
    shooterLerp.addPoint(35, 7.5);
    shooterLerp.addPoint(29, 8);
    shooterLerp.addPoint(23, 8.5);
    shooterLerp.addPoint(17, 9);

    driverTab = Shuffleboard.getTab("Driver");
    leftShooterSpeedEntry = driverTab.add("Left Shooter Speed", 9)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 12)).withPosition(2, 0).getEntry();
    rightShooterSpeedEntry = driverTab.add("Right Shooter Speed", 7)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 12)).withPosition(2, 1).getEntry();
    shooterModeEntry = driverTab.add("Shooter Mode", shooterMode.toString()).withPosition(2, 2).getEntry();
    pivotAdjustEntry = driverTab.add("Shooter Pivot Adjust", pivotAdjust).withPosition(3, 2).getEntry();
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

    shooterModeEntry.setString(shooterMode.toString());
    pivotAdjustEntry.setDouble(pivotAdjust);
  }

  public void shooterHold(){
    double shooterVoltage = shooterLerp.interpolate(calculatePivotAngle());

    if(DriverStation.isAutonomousEnabled()){
      leftShooter.set(0.7);
      rightShooter.set(0.6);
    }
    else if(RobotContainer.driverController.getLeftTriggerAxis() >= 0.95){ //Amp
      leftController.setReference(
        3.9,
        ControlType.kVoltage,
        0);

      rightController.setReference(
        3.8,
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
    else if(shooterMode == ShooterMode.Speaker){
      leftController.setReference(
        6.5,
        ControlType.kVoltage,
        0);

      rightController.setReference(
        4.5,
        ControlType.kVoltage,
        0);
    }
    else if(shooterMode == ShooterMode.Manual){
      leftController.setReference(
        leftShooterSpeedEntry.getDouble(9),
        ControlType.kVoltage,
        0);

      rightController.setReference(
        rightShooterSpeedEntry.getDouble(7),
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
    setAutoMode();
    leftShooter.set(speed);
    rightShooter.set(speed);
  }

  public void pivotHold(){
    if(zeroing){
      pivot.set(-0.15);
    }
    else if(RobotContainer.driverController.getLeftTriggerAxis() >= 0.95){
      pivotController.setReference(
        ShooterConstants.AMP_PIVOT_POSITION,
        ControlType.kPosition,
        0);

      pivotPosition = ShooterConstants.AMP_PIVOT_POSITION;
    }
    else if(shooterMode == ShooterMode.Passing){
      pivotController.setReference(
        ShooterConstants.PASSING_PIVOT_POSITION,
        ControlType.kPosition,
        0);

      pivotPosition = ShooterConstants.PASSING_PIVOT_POSITION;
    }
    else if(shooterMode == ShooterMode.Speaker){
      pivotController.setReference(
        ShooterConstants.SPEAKER_PIVOT_POSITION,
        ControlType.kPosition,
        0);

      pivotPosition = ShooterConstants.SPEAKER_PIVOT_POSITION;
    }
    else{
      if(shooterMode == ShooterMode.Auto && hasPriorityTarget()){
        setPivotAngle(calculatePivotAngle());
      }

      pivotController.setReference(
        pivotPosition + pivotAdjust,
        ControlType.kPosition,
        0);
    }

    if(RobotContainer.opController.getPOV() == 0){
      pivotAdjust += 0.1;
    }
    else if(RobotContainer.opController.getPOV() == 180){
      pivotAdjust -= 0.1;
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
