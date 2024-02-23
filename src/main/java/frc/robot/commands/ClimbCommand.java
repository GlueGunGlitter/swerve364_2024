// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ClimbCommand extends Command {
  boolean climbLeft = false;
  boolean climbRight = false;

  /** Creates a new ClimbCommand. */
  public ClimbCommand() {
    addRequirements(RobotContainer.m_ClimbSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.xboxController.getRightTriggerAxis() > 0.4) {
      RobotContainer.m_ClimbSubsystem.setSpeed(Robot.climbRightMotorSpeed.getDouble(0),
          Robot.climbLeftNotorSpeed.getDouble(0));
    } else {
      RobotContainer.m_ClimbSubsystem.stopMotor();
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