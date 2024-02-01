// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  WPI_VictorSPX intakeLowerMoter = new WPI_VictorSPX(Constants.IntakeConstants.INTAKE_LOWER_MOTOR_PORT);
  WPI_VictorSPX intakeHigherMotor = new WPI_VictorSPX(Constants.IntakeConstants.INTAKE_HIGHER_MOTOR_PORT);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeHigherMotor.setInverted(true);
    intakeLowerMoter.setInverted(true);
  }

  public void setSpeed(double speedToLowerMoter, double speedToHigherMotor) {
    intakeLowerMoter.set(speedToLowerMoter);
    intakeHigherMotor.set(speedToHigherMotor);
  }

  public void stopMotors() {
    intakeHigherMotor.stopMotor();
    intakeLowerMoter.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}