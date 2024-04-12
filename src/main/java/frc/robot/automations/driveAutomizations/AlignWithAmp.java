// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations.driveAutomizations;

import java.sql.Driver;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignWithAmp extends PIDCommand {
  Swerve swerve;

  /** Creates a new RorateToAngel. */
  public AlignWithAmp(Swerve swerve, DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier rotationSup) {
    super(
        // The controller that the command will use
        new PIDController(0.0055, 0, 0.00055),
        // This should return the measurement
        () -> Math.IEEEremainder(swerve.getHeading().getDegrees(), 360),
        // This should return the setpoint (can also be a constant)
        () -> wantedAngle(),
        // This uses the output
        output -> {
          double translationVal = MathUtil.applyDeadband(translationX.getAsDouble(), Constants.stickDeadband);
          double strafeVal = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.stickDeadband);
          double rotationVal = -MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

          /* Drive */
          swerve.drive(
              new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
              -output * Constants.Swerve.maxAngularVelocity,
              false,
              true);
        },
        swerve);

    this.swerve = swerve;
  };

  private static int wantedAngle() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return 90;
    } else {
      return -90;
    }
  }
  // Use addRequirements() here to declare subsystem dependencies.
  // Configure additional PID options by calling `getController` here.

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.IEEEremainder(swerve.getHeading().getDegrees(), 360) < 90
        + Constants.AutomationsConstants.TOLERANCE_OF_DGREE
        && Math.IEEEremainder(swerve.getHeading().getDegrees(), 360) > 90
            - Constants.AutomationsConstants.TOLERANCE_OF_DGREE) {
      return true;

    } else {
      return false;
    }
  }
}