// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.drivers.PearadoxSparkMax;
import frc.lib.util.SmarterDashboard;
import frc.robot.Constants.TransportConstants;
import frc.robot.RobotContainer;

public class Transport extends SubsystemBase {
  private PearadoxSparkMax topTransport;
  private PearadoxSparkMax botTransport;

  private DigitalInput irSensor;
  private Debouncer debouncer;

  private boolean isHolding = true;
  private boolean rumbled = false;

  private static final Transport transport = new Transport();

  public static Transport getInstance(){
    return transport;
  }

  /** Creates a new Transport. */
  public Transport() {
    topTransport = new PearadoxSparkMax(TransportConstants.TOP_TRANSPORT_ID, MotorType.kBrushless, IdleMode.kBrake, 40, false);
    botTransport = new PearadoxSparkMax(TransportConstants.BOT_TRANSPORT_ID, MotorType.kBrushless, IdleMode.kBrake, 40, false);

    irSensor = new DigitalInput(TransportConstants.IR_SENSOR_CHANNEL);
    debouncer = new Debouncer(0.1, DebounceType.kFalling);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmarterDashboard.putBoolean("Ir Sensor", hasNote(), "Transport");

    if(isHolding){
      if(hasNote()){
        transportStop();
      }
      else{
        transportHold();
      }
    }

    if(!rumbled && hasNote()){
      CommandScheduler.getInstance().schedule(rumbleController());
      rumbled = true;
    }
    if(rumbled && !hasNote()){
      rumbled = false;
    }
  }

  public void transportHold(){
    topTransport.set(0.6);
    botTransport.set(0.6);
  }

  public void transportOut(){
    topTransport.set(-0.5);
    botTransport.set(-0.5);
  }

  public void transportStop(){
    topTransport.set(0);
    botTransport.set(0);
  }

  public void transportShoot(){
    topTransport.set(1);
    botTransport.set(1);
  }

  public void setBrakeMode(boolean brake){
    if(brake){
      topTransport.setIdleMode(IdleMode.kBrake);
      botTransport.setIdleMode(IdleMode.kBrake);
    }
    else{
      topTransport.setIdleMode(IdleMode.kCoast);
      botTransport.setIdleMode(IdleMode.kCoast);
    }
  }

  public boolean hasNote(){
    return debouncer.calculate(!irSensor.get());
  }

  public void setHolding(boolean isHolding){
    this.isHolding = isHolding;
  }

  public Command rumbleController(){
    return new InstantCommand(() -> RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0.5))
      .andThen(new WaitCommand(0.75))
      .andThen(new InstantCommand(() -> RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0)));
  }
}
