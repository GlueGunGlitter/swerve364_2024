// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  WPI_TalonSRX climbRightMotor = new WPI_TalonSRX(Constants.ClimbConstants.CLIMB_RIGHT_MOTOR_PORT);
  WPI_TalonSRX climbLeftMotor = new WPI_TalonSRX(Constants.ClimbConstants.CLIMB_LEFT_MOTOR_PORT);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
  }

  public void setSpeed(double climbRightMotorSpeed, double climbLeftMotorSpeed) {
    climbRightMotor.set(climbRightMotorSpeed);
    climbLeftMotor.set(climbLeftMotorSpeed);
  }

  public void stopMotor() {
    climbLeftMotor.set(0);
    climbRightMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}