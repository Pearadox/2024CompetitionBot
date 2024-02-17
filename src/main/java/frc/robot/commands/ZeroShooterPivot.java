// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ZeroShooterPivot extends Command {
  private Shooter shooter = Shooter.getInstance();

  /** Creates a new ZeroShooterPivot. */
  public ZeroShooterPivot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setPivot(0.15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setPivot(0);
    shooter.resetPivotEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.getPivotCurrent() > 10;
  }
}
