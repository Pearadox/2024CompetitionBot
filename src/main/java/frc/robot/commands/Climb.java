// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
  
  private Climber climber = Climber.getInstance();

  public Climb() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  if(climber.zeroing) {
    climber.zeroClimber();
  }else{
      if (climber.getClimbMode() == Climber.ClimbMode.Climbed) {
        climber.setClimberPosition(ClimberConstants.CLIMBED_ROT);
      } else if (climber.getClimbMode() == Climber.ClimbMode.Climbing) {
        climber.setClimberPosition(ClimberConstants.CLIMBING_ROT);
      } else {
        climber.setClimberPosition(0);
      } 
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
