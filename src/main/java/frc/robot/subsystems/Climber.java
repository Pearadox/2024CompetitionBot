// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkFlex;
import frc.lib.util.SmarterDashboard;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  private static PearadoxSparkFlex climbMotor;
  private static SparkPIDController climberController;
  private static RelativeEncoder climberEncoder;

  private static Climber climber = new Climber();

  private static ClimbMode climbMode = ClimbMode.Normal;

  private double climberAdjust = 0;
  public boolean zeroing = false;
  
  public enum ClimbMode{
    Normal, Climbing, Climbed
  }

  public static Climber getInstance() {
    return climber;
  }

  public Climber() {
    climbMotor = new PearadoxSparkFlex(
      ClimberConstants.CLIMBER_ID, MotorType.kBrushless, IdleMode.kBrake, 45, false); //TOOD: change current limit?
    climberController = climbMotor.getPIDController();
    climberEncoder = climbMotor.getEncoder();

  }

  @Override
  public void periodic() {
    SmarterDashboard.putNumber("Climber Position", getClimberPosition(), "Climber");
    SmarterDashboard.putString("ClimbMode", getClimbMode().toString(), "Climber");
    SmarterDashboard.putNumber("Climber Adjust", climberAdjust, "Climber");

    if(climbMode == climbMode.Normal){
      setPower(0);
    } else if(climbMode == climbMode.Climbing) {
      setPower(0.5);
    } else if(climbMode == climbMode.Climbed) {
      setPower(0);
    }
  }

  public void setNormalMode() {
    climbMode = climbMode.Normal;
  }

  public void setClimbingMode() {
    climbMode = climbMode.Climbing;
  }

  public void setClimbedMode() {
    climbMode = climbMode.Climbed;
  }

  public ClimbMode getClimbMode() {
    return climbMode;
  }

  public void setClimberPosition(double reference) {
    climberController.setReference(climberAdjust + reference, ControlType.kPosition, 0);
  }

  public void setPower(double power) {
    climbMotor.set(power);
  }

  public void setZeroing(boolean zeroing) {
    this.zeroing = zeroing;
  }

  public void zeroClimber() {
    climbMotor.set(-0.15); //TODO: change?
  }

  public double getClimberPosition() {
    return climberEncoder.getPosition();
  }
}
