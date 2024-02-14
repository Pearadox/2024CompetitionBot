// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkFlex;
import frc.lib.drivers.PearadoxSparkMax;
import frc.lib.util.LerpTable;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private PearadoxSparkFlex leftShooter;
  private PearadoxSparkMax rightShooter;
  
  private PearadoxSparkMax pivot;

  private RelativeEncoder pivotEncoder;

  private SparkPIDController leftController;
  private SparkPIDController rightController;
  private SparkPIDController pivotController;

  private LerpTable shooterLerp;

  private double pivotPosition;

  private boolean zeroing = false;

  private enum ShooterMode{
    Auto, Speaker
  }

  private ShooterMode shooterMode = ShooterMode.Speaker;

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

    shooterLerp = new LerpTable();

    //SHOOTER LOOKUP TABLE: (distance, voltage)
    shooterLerp.addPoint(0, 0);

    SmartDashboard.putNumber("Left Shooter Speed (Voltage)", 6);
    SmartDashboard.putNumber("Right Shooter Speed (Voltage)", 6);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Pivot Position", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Pivot Intended Position", pivotPosition);
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
        SmartDashboard.getNumber("Left Shooter Speed (Voltage)", 6),
        CANSparkMax.ControlType.kVoltage,
        0);
  
      rightController.setReference(
        SmartDashboard.getNumber("Right Shooter Speed (Voltage)", 6),
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
}
