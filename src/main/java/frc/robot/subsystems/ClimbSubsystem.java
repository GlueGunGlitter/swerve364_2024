// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  WPI_TalonSRX climbRightMotor = new WPI_TalonSRX(Constants.ClimbConstants.CLIMB_RIGHT_MOTOR_PORT);
  WPI_TalonSRX climbLeftMotor = new WPI_TalonSRX(Constants.ClimbConstants.CLIMB_LEFT_MOTOR_PORT);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    climbLeftMotor.configContinuousCurrentLimit(30);
    climbRightMotor.configContinuousCurrentLimit(30);
  }

  public void setSpeed(double climbRightMotorSpeed, double climbleftMotorSpeed) {
    climbRightMotor.set(climbRightMotorSpeed);
    climbLeftMotor.set(climbleftMotorSpeed);
  }

  public void stopMotor() {
    climbLeftMotor.set(0);
    climbRightMotor.set(0);
  }

  public Command setSpeedCommand(double climbRightMotorSpeed, double climbleftMotorSpeed){
    return new RunCommand( () -> setSpeed(climbRightMotorSpeed, climbleftMotorSpeed), this);
  }

  public Command stopMotorsCommand(){
    return new RunCommand( () -> stopMotor(), this);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}