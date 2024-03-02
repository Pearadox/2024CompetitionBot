// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.AmpBarConstants;

public class AmpBar extends SubsystemBase {
  private PearadoxSparkMax ampBar;

  private RelativeEncoder ampBarEncoder;
  private SparkPIDController ampBarController;

  public enum AmpBarMode{
    Stowed, Deployed
  }

  public AmpBarMode ampBarMode = AmpBarMode.Stowed;

  private static final AmpBar AMP_BAR = new AmpBar();

  public static AmpBar getInstance(){
    return AMP_BAR;
  }

  /** Creates a new AmpBar. */
  public AmpBar() {
    ampBar = new PearadoxSparkMax(AmpBarConstants.AMP_BAR_ID, MotorType.kBrushless, IdleMode.kBrake, 40, false, 
      AmpBarConstants.AMP_BAR_kP, AmpBarConstants.AMP_BAR_kI, AmpBarConstants.AMP_BAR_kD, 
      AmpBarConstants.AMP_BAR_MIN_OUTPUT, AmpBarConstants.AMP_BAR_MAX_OUTPUT);

    ampBarEncoder = ampBar.getEncoder();
    ampBarController = ampBar.getPIDController();
  }

  @Override
  public void periodic() {
    if(ampBarMode == AmpBarMode.Deployed){
      ampBarController.setReference(
        AmpBarConstants.DEPLOYED_ROT,
        ControlType.kPosition,
        0);
    }
    else{
      ampBarController.setReference(
        0,
        ControlType.kPosition,
        0);
    }

    SmartDashboard.putNumber("Amp Bar Position", ampBarEncoder.getPosition());
  }

  public void setStowedMode(){
    ampBarMode = AmpBarMode.Stowed;
  }

  public void setDeployedMode(){
    ampBarMode = AmpBarMode.Deployed;
  }
}
