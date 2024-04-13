// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automations.driveAutomizations;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TransportationSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpAssist extends PIDCommand {
  /** Creates a new Test. */

  // create a timer
  private static Timer startTransportionTimer = new Timer();

  public AmpAssist(Swerve swerve, DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier rotationSup, TransportationSubsystem transportationSubsystem, ShooterSubsystem shooterSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(Constants.AmpAssistConstants.KP, 0, Constants.AmpAssistConstants.KD),

        // This should return the measurement
        // distanceFromTheMiddleOfTheAprilTag() retern how much far the camera is from the middle of the aprilTag
        () -> RobotContainer.aprilTag_Vision.distanceFromTheMiddleOfTheAprilTag(wantedAprilTagID()),
        // This should return the setpoint (can also be a constant)
        // set the setpoint to 0 because if distance is 0 the aprilTag will be in the middel of the camera
        // because of that the AMP will be face the meddel of the robot
        () -> 0,
        // This uses the output
        output -> {

          // apply deadband
          double strafeVal = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.stickDeadband);
          double rotationVal = -MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);


          // if the camera see aprilTag the robot will center himself and drive forword in robot reltive mode
          // else the will only drive forword in robot reltive mode
          if (RobotContainer.aprilTag_Vision.seesAprilTags()) {

            swerve.drive(
                new Translation2d(-Constants.AmpAssistConstants.SPEED_WHILE_DRIVE_TO_AMP_WHENE_SEE_APRILTAG,
                    output)
                    .times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                false,
                true);

            // keeps trac of how much the tobot is close to the is from the aprilTag  
            RobotContainer.destenceFromAprilTag = RobotContainer.aprilTag_Vision.distanceFromAprilTag(wantedAprilTagID());

          } else {
            swerve.drive(
                new Translation2d(-Constants.AmpAssistConstants.SPEED_WHILE_DRIVE_TO_AMP_WHENE_NOT_SEE_APRILTAG,
                strafeVal)
                .times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                false,
                true);
            

            // if the robot is close enough it start shooter and strat the timer
            if (RobotContainer.destenceFromAprilTag < Constants.AmpAssistConstants.METERS_OF_START_SHOTER) {
              shooterSubsystem.shootDown(0.5, 0.4);
              startTransportionTimer.start();

              // if the timer get to desire time it will start transportation
              if (startTransportionTimer
                  .hasElapsed(Constants.AmpAssistConstants.TIME_DELAY_BETWEEN_SHOOTER_AND_TRANSPORTATION)) {
                transportationSubsystem.setSpeed(0.8, 0.8, 0.7, 0.7);
              }
            }

            // if the driver stop press B the trace of the destence from the aprilTag will get back to is default valaue which is 10
            if (RobotContainer.xboxController.getBButtonReleased()) {
              RobotContainer.destenceFromAprilTag = 10;
            }
          }


          /* 
           if the variable that keeps trace of the destence from the aprilTag is bigger then desire anount
           it will set the timer back to 0
          */
          if (RobotContainer.destenceFromAprilTag > Constants.AmpAssistConstants.METERS_OF_START_SHOTER || RobotContainer.xboxController.getBButtonReleased()) {
            startTransportionTimer.reset();
          }

        },
        swerve);

  };
  // get which aprilTag the AMP of your alliance have
  private static int wantedAprilTagID() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return 6;
    } else {
      return 5;
    }
  }
  
  // Use addRequirements() here todeclare subsystem dependencies.
  // Configure additional PID options by calling `getController` here.

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}