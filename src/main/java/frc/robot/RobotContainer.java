// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.drivers.vision.PoseEstimation;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.IntakeHold;
import frc.robot.commands.Outtake;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterHold;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.AmpBar;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain drivetrain = Drivetrain.getInstance();
  public static final Intake intake = Intake.getInstance();
  public static final Transport transport = Transport.getInstance();
  public static final Shooter shooter = Shooter.getInstance();
  public static final AmpBar ampBar = AmpBar.getInstance();

  //Driver Controls
  public static final XboxController driverController = new XboxController(IOConstants.DRIVER_CONTROLLER_PORT);

  private final JoystickButton resetHeading_Start = new JoystickButton(driverController, XboxController.Button.kStart.value);
  private final JoystickButton ampBar_A = new JoystickButton(driverController, XboxController.Button.kA.value);
  private final JoystickButton shoot_RB = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
  private final JoystickButton zeroingShooter_X = new JoystickButton(driverController, XboxController.Button.kX.value);
  private final JoystickButton outtake_B = new JoystickButton(driverController, XboxController.Button.kB.value);
  private final JoystickButton turnToApril_LB = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);

  //Operator Controls
  public static final XboxController opController = new XboxController(IOConstants.OP_CONTROLLER_PORT);
  
  private final JoystickButton shooterAutoMode_A = new JoystickButton(opController, XboxController.Button.kA.value);
  private final JoystickButton shooterPassingMode_Y = new JoystickButton(opController, XboxController.Button.kY.value);
  private final JoystickButton shooterManualMode_B = new JoystickButton(opController, XboxController.Button.kB.value);
  private final JoystickButton shooterSpeakerMode_X = new JoystickButton(opController, XboxController.Button.kX.value);

  //Pose Estimation
  public static final PoseEstimation poseEstimation = new PoseEstimation();
  public static AprilTagFieldLayout aprilTagFieldLayout;

  //Shuffleboard
  public static final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
  public static final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. 
   * @throws IOException */
  public RobotContainer() throws IOException {
    registerNamedCommands();
    configureBindings();
    setDefaultCommands();
    configureAutoTab();

    aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      
    HttpCamera httpCamera = new HttpCamera("Limelight", "http://10.54.14.11:5800");
    CameraServer.addCamera(httpCamera);
    driverTab.add(httpCamera).withSize(6, 4).withPosition(4, 0);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //Driver Buttons
    resetHeading_Start.onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));
    ampBar_A.whileTrue(new RunCommand(() -> ampBar.setDeployedMode())).onFalse(new InstantCommand(() -> ampBar.setStowedMode()));
    zeroingShooter_X.whileTrue(new RunCommand(() -> shooter.setZeroing(true)))
      .onFalse(new InstantCommand(() -> shooter.setZeroing(false))
      .andThen(new InstantCommand(() -> shooter.resetPivotEncoder())));
    shoot_RB.whileTrue(new Shoot());
    outtake_B.whileTrue(new Outtake());
    turnToApril_LB.onTrue(new InstantCommand(() -> drivetrain.setAlignMode()))
      .onFalse(new InstantCommand(() -> drivetrain.setNormalMode()));

    //Operator Buttons
    shooterAutoMode_A.onTrue(new InstantCommand(() -> shooter.setAutoMode()));
    shooterManualMode_B.onTrue(new InstantCommand(() -> shooter.setManualMode()));
    shooterPassingMode_Y.onTrue(new InstantCommand(() -> shooter.setPassingMode()));
    shooterSpeakerMode_X.onTrue(new InstantCommand(() -> shooter.setSpeakerMode()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void registerNamedCommands(){
    NamedCommands.registerCommand("Stop Modules", new InstantCommand(() -> drivetrain.stopModules()));
    NamedCommands.registerCommand("Auto Align", new AutoAlign().withTimeout(0.3));
    NamedCommands.registerCommand("Shoot", new Shoot().withTimeout(0.2));
    NamedCommands.registerCommand("Source Set Pivot Position", new InstantCommand(() -> shooter.setPivotPosition(9.05)));
    NamedCommands.registerCommand("Set Shooter Auto", new InstantCommand(() -> shooter.setShooterAuto(0.85)));
    NamedCommands.registerCommand("Reset Heading", new InstantCommand(drivetrain::zeroHeading, drivetrain));
    NamedCommands.registerCommand("7 Note Set Pivot Position", new InstantCommand(() -> shooter.setPivotPosition(11.5)));
  }

  public void setDefaultCommands(){
    drivetrain.setDefaultCommand(new SwerveDrive());
    intake.setDefaultCommand(new IntakeHold());
    shooter.setDefaultCommand(new ShooterHold());
  }

  private void configureAutoTab() {
    autoChooser = AutoBuilder.buildAutoChooser("Two Meters");
    autoTab.add("Auto Chooser", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 1).withPosition(4, 0);
  }
}
