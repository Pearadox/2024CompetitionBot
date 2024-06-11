// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class AutoAlign extends Command {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private Shooter shooter = Shooter.getInstance();
  private double time;
  private Timer timer;
  private final double delay = 1.0;

  /** Creates a new AutoAlign. */
  public AutoAlign() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.restart();
    timer.start();
    time = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drivetrain.swerveDrive(
    //   0,
    //   0, 
    //   -drivetrain.getAlignSpeed(),
    //   true,
    //   new Translation2d(),
    //   false);
    shooter.setStageMode();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() > time + delay;
  }
}
