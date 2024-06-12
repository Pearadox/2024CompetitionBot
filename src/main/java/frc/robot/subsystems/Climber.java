// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.lib.util.SmarterDashboard;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  private static PearadoxSparkMax climber;

  private static SparkPIDController climberController;

  private static RelativeEncoder climberEncoder;
  
  private static Climber climb = new Climber();

  public static Climber getInstance() {
    return climb;
  }

  public Climber() {
    climber = new PearadoxSparkMax(ClimberConstants.CLIMBER_ID, MotorType.kBrushless, IdleMode.kBrake, 50, false);
    climberController = climber.getPIDController();
    climberEncoder = climber.getEncoder();
  
  }

  @Override
  public void periodic() {
    SmarterDashboard.putNumber("Climber Position", getPosition(), "Climber");
  }

  public double getPosition() {
    return climberEncoder.getPosition();
  }

  public void setClimberReference(double reference) {
    climberController.setReference(reference, ControlType.kPosition, 0);
  }

  public void resetEncoder() {
    climberEncoder.setPosition(0);
  }
  
}

