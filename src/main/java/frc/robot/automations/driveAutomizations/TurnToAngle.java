package frc.robot.automations.driveAutomizations;

// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {
  /** Creates a new TrunToAngle. */

  public TurnToAngle(Swerve m_swerve) {
    super(
        // The controller that the command will use
        new PIDController(1, 0.0, 0.0),
        // This should return the measurement
        () -> m_swerve.getGyroYaw().getDegrees(),
        // This should return the setpoint (can also be a constant)
        90,
        // This uses the output
        output -> m_swerve.drive(new Translation2d(0, 0), -output, true, true),
        m_swerve);

    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}