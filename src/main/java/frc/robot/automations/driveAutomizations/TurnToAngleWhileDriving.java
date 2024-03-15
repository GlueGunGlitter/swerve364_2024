package frc.robot.automations.driveAutomizations;

import java.util.function.DoubleSupplier;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.MathUtil;

// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAngleWhileDriving extends PIDCommand {
  /** Creates a new TrunToAngle. */

  public TurnToAngleWhileDriving(
      Swerve m_swerve, DoubleSupplier angle,
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotationSup) {
    super(
        // The controller that the command will use
        new PIDController(0.1, 0.1, 0.1),
        // This should return the measurement
        () -> m_swerve.getGyroYaw().getDegrees(),
        // This should return the setpoint (can also be a constant)
        angle,
        // This uses the output
        output -> {

          double translationVal = -MathUtil.applyDeadband(translationX.getAsDouble(), Constants.stickDeadband);
          double strafeVal = -MathUtil.applyDeadband(translationY.getAsDouble(), Constants.stickDeadband);
          double rotationVal = -MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
          System.out.println(output / 50);

          /* Drive */
          if (Robot.seesNote()) {
            m_swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                output * Constants.Swerve.maxAngularVelocity,
                true,
                true);
            // }
          } else {
            m_swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                true,
                true);
          }
        },
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