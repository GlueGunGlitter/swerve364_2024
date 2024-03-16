// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class enableForwardTransportation extends SequentialCommandGroup {
  /** Creates a new autoCommand. */
  public enableForwardTransportation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand0+(), new BarCommand());
    addCommands(
        new InstantCommand(() -> RobotContainer.m_TransportationSubsystem.setSpeed(0.7, 0.7, 0.5, 0.5))); // #TODO://up
                                                                                                          // 0.7
                                                                                                          // transpotation
                                                                                                          // Change
    // the value
  }
}
