// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations.driveAutomizations;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAssist extends PIDCommand {
  /** Creates a new Test. */
  public AimAssist(Swerve swerve, DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier rotationSup) {
    super(
        // The controller that the command will use
        new PIDController(Constants.AimAssistConstans.KP, 0, Constants.AimAssistConstans.KD),
        // This should return the measurement 
        // getRobotToNoteYaw() retern the horizontal engel from the camera to the note 
        () -> RobotContainer.note_vision.getRobotToNoteYaw(),
        // This should return the setpoint (can also be a constant)
        // set the setpoint to 0 because if engel is 0 the note will be in the middel of the camera
        // because of that the note will be in the meddel of the robot
        () -> 0,
        // This uses the output
        output -> {

          // apply deadband
          double translationVal = MathUtil.applyDeadband(translationX.getAsDouble(), Constants.stickDeadband);
          double strafeVal = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.stickDeadband);
          double rotationVal = -MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

          // if the camera see not it will rotate to the note to get close as possible to the setpoint
          // else it will the let the driver drive in the regular way
          if (RobotContainer.note_vision.seesNote()) {

            // if the driver press y the robot will drive the note
            // else the robot will only rotate to the note
            if (RobotContainer.xboxController.getYButton()) {
              swerve.drive(
                  new Translation2d(Constants.AimAssistConstans.SPEED_WHILE_DRIVE_TO_NOTE, -strafeVal)
                      .times(Constants.Swerve.maxSpeed),
                  -output * Constants.Swerve.maxAngularVelocity,
                  false,
                  true);
            } else {
              swerve.drive(
                  new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                  -output * Constants.Swerve.maxAngularVelocity,
                  true,
                  true);
            }
          } else {
            swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                true,
                true);
          }

        },
        swerve);
  };
  // Use addRequirements() here to declare subsystem dependencies.
  // Configure additional PID options by calling `getController` here.

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}