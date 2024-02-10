// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.IntakeConstants;
// import frc.robot.Constants.TransportConstants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  private PearadoxSparkMax utbRoller;
  private PearadoxSparkMax otbRoller;
  // private PearadoxSparkMax btmIntake;
  // private PearadoxSparkMax topIntake;

  // private PearadoxSparkMax otbPivot;

  // private RelativeEncoder otbPivotEncoder;
  // private SparkPIDController otbPivotController;

  private double intakeAdjust = 0;
  private boolean deployed = false;
  private boolean zeroing = false;

  private static final Intake INTAKE = new Intake();

  private double power;

  public static Intake getInstance(){
    return INTAKE;
  }

  /** Creates a new Intake. */
  public Intake() {
    utbRoller = new PearadoxSparkMax(IntakeConstants.UTB_ROLLER_ID, MotorType.kBrushless, IdleMode.kBrake, 35, false); //TODO: set intake inversion
    otbRoller = new PearadoxSparkMax(IntakeConstants.OTB_ROLLER_ID, MotorType.kBrushless, IdleMode.kBrake, 35, false);
    //otbPivot = new PearadoxSparkMax(IntakeConstants.OTB_PIVOT_ID, MotorType.kBrushless, IdleMode.kBrake, 40, true,
      //IntakeConstants.PIVOT_kP, IntakeConstants.PIVOT_kI, IntakeConstants.PIVOT_kD, 
      //IntakeConstants.PIVOT_MIN_OUTPUT, IntakeConstants.PIVOT_MAX_OUTPUT);

    //otbPivotEncoder = otbPivot.getEncoder();
    //otbPivotController = otbPivot.getPIDController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Pivot Position", otbPivotEncoder.getPosition());
    SmartDashboard.putBoolean("Deployed", deployed);
    //SmartDashboard.putNumber("Pivot Current", otbPivot.getOutputCurrent());

    //Logger.recordOutput("Intake/Pivot Position", otbPivotEncoder.getPosition());
    Logger.recordOutput("Intake/Deployed", deployed);
    //Logger.recordOutput("Intake/Pivot Current", otbPivot.getOutputCurrent());

    if(DriverStation.isTeleopEnabled()){
      if(RobotContainer.driverController.getLeftTriggerAxis() > 0.5){
        deployed = true;
      }
      else{
        deployed = false;
      }
    }
  }

  public void utbIntakeIn(double power){
    double adjustedPower = power * (3./8);
    utbRoller.set(adjustedPower);
    otbRoller.set(adjustedPower);
    SmartDashboard.putNumber("BTM and TOP Roller Power", adjustedPower);
    this.power = power;
  }

  public void utbIntakeStop(){
    utbRoller.set(0);
  }

  public void otbIntakeIn(){
    otbRoller.set(deployed ? 0.5 : 0); //TODO: set otb intake speed
  }

  public void otbIntakeStop(){
    otbRoller.set(0);
  }

  public double getPower(){
    return power;
  }

  public void pivotHold(){
    /*if(zeroing){
      otbPivot.set(-0.25);
    }
    else if(deployed){
      otbPivotController.setReference(IntakeConstants.DEPLOYED_ROT + intakeAdjust, CANSparkMax.ControlType.kPosition, 0);
    }
    else{
      otbPivotController.setReference(0 + intakeAdjust, CANSparkMax.ControlType.kPosition, 0);
    }*/
  }

  public void intakeToggle(){
    deployed = !deployed;
  }

  public void intakeAdjustUp(){
    intakeAdjust += 0.5;
  }

  public void intakeAdjustDown(){
    intakeAdjust -= 0.5;
  }

  public void setZeroing(boolean zeroing){
    this.zeroing = zeroing;
  }

  public boolean isDeployed(){
    return deployed;
  }

  public void resetOtbPivotEncoder(){
   // otbPivotEncoder.setPosition(0);
  }
}
