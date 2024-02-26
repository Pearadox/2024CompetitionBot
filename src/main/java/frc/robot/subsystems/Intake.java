// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private PearadoxSparkMax utbRoller;

  private static final Intake INTAKE = new Intake();

  public static Intake getInstance(){
    return INTAKE;
  }

  /** Creates a new Intake. */
  public Intake() {
    utbRoller = new PearadoxSparkMax(IntakeConstants.UTB_ROLLER_ID, MotorType.kBrushless, IdleMode.kCoast, 35, true);
  }

  @Override
  public void periodic() {}

  public void utbIntakeIn(){
    utbRoller.set(1);
  }

  public void utbIntakeOut(){
    utbRoller.set(-0.7);
  }

  public void utbIntakeStop(){
    utbRoller.set(0);
  }
}
