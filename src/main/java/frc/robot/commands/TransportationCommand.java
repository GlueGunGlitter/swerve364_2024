// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TransportationCommand extends Command {
  boolean workUP = false;
  boolean workDown = false;

  /** Creates a new IntakkCommand. */
  public TransportationCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_TransportationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.xboxController.getRightBumperPressed()) {
      workUP = !workUP;
    }
    if (RobotContainer.xboxController.getXButtonPressed()) {
      workDown = !workDown;
    }
    if (workUP == true && workDown == true) {
      workUP = false;
    }
    if (workUP) {
      RobotContainer.m_TransportationSubsystem.setSpeed(Robot.intakeLowerMotorSpeed.getDouble(0),
          Robot.intakeHigherMotorSpeed.getDouble(0), Robot.transportationMotorSpeed.getDouble(0));
    } else if (workDown) {
      RobotContainer.m_TransportationSubsystem.setSpeed(Robot.intakeLowerMotorSpeed.getDouble(0)
          * -1,
          Robot.intakeHigherMotorSpeed.getDouble(0) * -1,
          Robot.transportationMotorSpeed.getDouble(0) * -1);
    } else {
      RobotContainer.m_TransportationSubsystem.stopMotors();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
