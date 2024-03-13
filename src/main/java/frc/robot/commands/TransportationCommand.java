// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.CANifier.PWMChannel;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.util.Color;
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
    // buffer = new AddressableLEDBuffer(1000);
    // color = Color.kHotPink;
    // channal = new AddressableLED(3);
    // channal.start();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.isTeleop()) {

      // channal.setData(buffer);
      if (RobotContainer.xboxController.getRightBumperPressed()) {
        workUP = !workUP;
        Robot.workTransportation.setBoolean(workUP);
      }
      if (RobotContainer.xboxController.getXButtonPressed()) {
        workDown = !workDown;
      }
      if (workUP == true && workDown == true) {
        workUP = false;
      }
      if (workUP) {
        RobotContainer.m_TransportationSubsystem.setSpeed(Robot.intakeLowerMotorSpeed.getDouble(0),
            Robot.intakeHigherMotorSpeed.getDouble(0), Robot.transportationMotorOneSpeed.getDouble(0),
            Robot.transportationMotorTwoSpeed.getDouble(0));
        // color = Color.kGreen;

      } else if (workDown) {
        RobotContainer.m_TransportationSubsystem.setSpeed(Robot.intakeLowerMotorSpeed.getDouble(0)
            * -1,
            Robot.intakeHigherMotorSpeed.getDouble(0) * -1,
            Robot.transportationMotorOneSpeed.getDouble(0) * -1,
            Robot.transportationMotorTwoSpeed.getDouble(0) * -1);

        // color = Color.kAqua;

      } else {
        RobotContainer.m_TransportationSubsystem.stopMotors();

      }
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
