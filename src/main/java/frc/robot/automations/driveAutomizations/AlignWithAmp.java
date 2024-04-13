// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations.driveAutomizations;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
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
        new PIDController(Constants.AlignWithAmpConstans.KP, 0, Constants.AlignWithAmpConstans.KD),

        // This should return the measurement
        // Math.IEEEremainder(swerve.getHeading().getDegrees(), 360) retern the robot engel and put it in the range of 180 and -180
        () -> Math.IEEEremainder(swerve.getHeading().getDegrees(), 360),
        // This should return the setpoint (can also be a constant)
        /// set the setpoint to the desire engel according to the alliance
        () -> wantedAngle(),
        // This uses the output
        output -> {

          // apply deadband
          double translationVal = MathUtil.applyDeadband(translationX.getAsDouble(), Constants.stickDeadband);
          double strafeVal = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.stickDeadband);
          
          // rotat the robot to the desire rengel
          swerve.drive(
              new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
              -output * Constants.Swerve.maxAngularVelocity,
              false,
              true);
        },
        swerve);
    // create an swerve that i can use outside of the constractor
    this.swerve = swerve;
  };

  // Use addRequirements() here to declare subsystem dependencies.
  // Configure additional PID options by calling `getController` here.

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // talls that the command as fineshd if the robot get in to a small tolerance from the setpint 
    if (Math.IEEEremainder(swerve.getHeading().getDegrees(), 360) < wantedAngle()
        + Constants.AlignWithAmpConstans.TOLERANCE_OF_DGREE
        && Math.IEEEremainder(swerve.getHeading().getDegrees(), 360) > wantedAngle()
            - Constants.AlignWithAmpConstans.TOLERANCE_OF_DGREE) {
      return true;

    } else {
      return false;
    }
  }

    // put the setpint to angel of the amp of your alliance
  private static int wantedAngle() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return 90;
    } else {
      return -90;
    }
    
  }
}