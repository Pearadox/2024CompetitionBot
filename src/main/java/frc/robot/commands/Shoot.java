// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Transport;

public class Shoot extends Command {
  private Transport transport = Transport.getInstance();

  /** Creates a new Shoot. */
  public Shoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transport.setHolding(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    transport.transportShoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transport.setHolding(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
