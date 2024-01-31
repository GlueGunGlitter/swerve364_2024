// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  public ShooterCommand() {
    addRequirements(RobotContainer.m_ShooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double staticMotorSpeed = Robot.staticShooterMotorSpeed.getDouble(0);
    double nonStaticMotorSpeed = Robot.nonStaticShooterMotorSpeed.getDouble(0);

    if (RobotContainer.xboxController.getAButton()) {
      RobotContainer.m_ShooterSubsystem.shootUp(nonStaticMotorSpeed, staticMotorSpeed);
    } else if (RobotContainer.xboxController.getBButton()) {
      RobotContainer.m_ShooterSubsystem.shootDown(nonStaticMotorSpeed, staticMotorSpeed);
    } else {
      RobotContainer.m_ShooterSubsystem.stopMotors();
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
