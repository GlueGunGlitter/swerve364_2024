// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.TransportationSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TransportAllTime extends SequentialCommandGroup {
  TransportationSubsystem m_TransportationSubsystem;

  /** Creates a new TransportAllTime. */
  public TransportAllTime(TransportationSubsystem m_TransportationSubsystem) {
    this.m_TransportationSubsystem = m_TransportationSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(m_TransportationSubsystem.transportUpAutoCommand(4));
  }
}
