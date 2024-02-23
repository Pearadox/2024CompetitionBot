// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkFlex;
import frc.lib.drivers.PearadoxSparkMax;
import frc.lib.util.LerpTable;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;

public class Shooter extends SubsystemBase {
  private PearadoxSparkFlex leftShooter;
  private PearadoxSparkMax rightShooter;
  
  private PearadoxSparkMax pivot;

  private RelativeEncoder pivotEncoder;

  private SparkPIDController leftController;
  private SparkPIDController rightController;
  private SparkPIDController pivotController;

  private double pivotPosition;

  private boolean zeroing = false;

  private static final NetworkTable llTable = NetworkTableInstance.getDefault().getTable(VisionConstants.LL_NAME);

  private enum ShooterMode{
    Auto, Speaker
  }

  private ShooterMode shooterMode = ShooterMode.Speaker;

  private LerpTable pivotLerp;

  private static final Shooter SHOOTER = new Shooter();

  public static Shooter getInstance(){
    return SHOOTER;
  }

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

    pivotEncoder = pivot.getEncoder();
    
    leftController = leftShooter.getPIDController();
    rightController = rightShooter.getPIDController();
    pivotController = pivot.getPIDController();

    SmartDashboard.putNumber("Left Shooter Speed (Voltage)", 3);
    SmartDashboard.putNumber("Right Shooter Speed (Voltage)", 3);

    pivotLerp = new LerpTable();
    pivotLerp.addPoint(53, 19.8);
    pivotLerp.addPoint(48, 16.7);
    pivotLerp.addPoint(43, 13.6);
    pivotLerp.addPoint(38, 12.3);
    pivotLerp.addPoint(33, 10.1);
    pivotLerp.addPoint(28, 8.1);
    pivotLerp.addPoint(23, 6.45);
    pivotLerp.addPoint(18, 5.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Pivot Position", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Pivot Intended Position", pivotPosition);
    SmartDashboard.putNumber("Shooter Pivot Current", pivot.getOutputCurrent());
    SmartDashboard.putNumber("Shooter Pivot Intended Angle", calculatePivotAngle());
  }

  public void shooterHold(){ //TODO: set voltages for shooting
    if(shooterMode == ShooterMode.Auto){
      leftController.setReference(
        0,
        CANSparkMax.ControlType.kVoltage,
        0);
  
      rightController.setReference(
        0,
        CANSparkMax.ControlType.kVoltage,
        0);
    }
    else if(shooterMode == ShooterMode.Speaker){
      leftController.setReference(
        SmartDashboard.getNumber("Left Shooter Speed (Voltage)", 3),
        CANSparkMax.ControlType.kVoltage,
        0);
  
      rightController.setReference(
        SmartDashboard.getNumber("Right Shooter Speed (Voltage)", 3),
        CANSparkMax.ControlType.kVoltage,
        0);
    }
    else{
      leftController.setReference(
        0,
        CANSparkMax.ControlType.kVoltage,
        0);
  
      rightController.setReference(
        0,
        CANSparkMax.ControlType.kVoltage,
        0);
    }
  }

  public void pivotHold(){ //TODO: set position for pivot
    if(zeroing){
      pivot.set(-0.05);
    }
    else{
      pivotController.setReference(
        pivotPosition,
        CANSparkMax.ControlType.kPosition,
        0);
    }
  }

  public void setAutoMode(){
    shooterMode = ShooterMode.Auto;
  }

  public void setSpeakerMode(){
    shooterMode = ShooterMode.Speaker;
  }

  public void changePivotPosition(double change){
    pivotPosition += change;
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
    double[] botpose_targetspace = llTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    double x = Math.abs(botpose_targetspace[0]);
    double z = Math.abs(botpose_targetspace[2]);
    double hypot = Math.hypot(x, z);

    double angle = Math.atan((FieldConstants.SPEAKER_HEIGHT - ShooterConstants.FLOOR_TO_SHOOTER) / hypot);
    return Units.radiansToDegrees(angle);
  }

  public void setPivotAngle(double angle){
    pivotPosition = pivotLerp.interpolate(angle);
  }

  public boolean hasTarget(){
    return llTable.getEntry("tv").getDouble(0) == 1.0;
  }

  public void setPipeline(int index){
    llTable.getEntry("pipeline").setNumber(index);
  }
}
