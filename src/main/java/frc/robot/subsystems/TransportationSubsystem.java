// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class TransportationSubsystem extends SubsystemBase {
  CANSparkFlex intakeHigherMotor = new CANSparkFlex(Constants.TransportationConstants.INTAKE_HIGHER_MOTOR_PORT,
      MotorType.kBrushless);
  CANSparkFlex intakeLowerMotor = new CANSparkFlex(Constants.TransportationConstants.INTAKE_LOWER_MOTOR_PORT,
      MotorType.kBrushless);
  WPI_TalonSRX transportationMotor1 = new WPI_TalonSRX(Constants.TransportationConstants.TRANSPORTATION_MOTOR_PORT_ONE);
  WPI_TalonSRX transportationMotor2 = new WPI_TalonSRX(Constants.TransportationConstants.TRANSPORTATION_MOTOR_PORT_TWO);

  // AddressableLED m_led = new AddressableLED(3);

  // // Reuse buffer
  // // Default to a length of 60, start empty output
  // // Length is expensive to set, so only set it once, then just update data
  // AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);

  /** Creates a new IntakeSubsystem. */
  public TransportationSubsystem() {
    intakeHigherMotor.setInverted(true);
    intakeLowerMotor.setInverted(true);
    transportationMotor1.setInverted(true);
    transportationMotor2.setInverted(true);
    transportationMotor1.configContinuousCurrentLimit(30);
    transportationMotor2.configContinuousCurrentLimit(30);

    // m_led.setLength(m_ledBuffer.getLength());
    // m_led.setData(m_ledBuffer);
    // m_led.start();
  }

  public void setSpeed(double intakeLowerMotorSpeed, double intakeHigherMotorSpeed, double transportationMotorOneSpeed,
      double transportationMotorTwoSpeed) {
    intakeHigherMotor.set(intakeHigherMotorSpeed);
    intakeLowerMotor.set(intakeLowerMotorSpeed);
    transportationMotor1.set(transportationMotorOneSpeed);
    transportationMotor2.set(transportationMotorTwoSpeed);

  }

  public void stopMotors() {
    intakeHigherMotor.set(0);
    intakeLowerMotor.set(0);
    transportationMotor1.set(0);
    transportationMotor2.set(0);

    // for (var i = 0; i < m_ledBuffer.getLength(); i++) {

    // m_ledBuffer.setRGB(i, 0, 255, 0);
    // }
    // m_led.setData(m_ledBuffer);
  }

  public Command transportUpAotoCommand() {
    return this.run(() -> setSpeed(0, 0, 0.9, 0.9));
  }

  public Command transportDowmAotoCommand() {
    return this.run(() -> setSpeed(0, 0, -0.5, -0.5));
  }

  public Command transportUpHighSpeedAotoCommand() {
    return this.run(() -> setSpeed(0.7, 0.7, 0.9, 0.9));
  }

  public Command transportDownCommand() {
    return this.run(() -> setSpeed(Robot.intakeLowerMotorSpeed.getDouble(0) * -1,
        Robot.intakeHigherMotorSpeed.getDouble(0) * -1,
        Robot.transportationMotorOneSpeed.getDouble(0) * -1,
        Robot.transportationMotorTwoSpeed.getDouble(0) * -1));
  }

  public Command transportUpCommand() {
    return this.run(() -> setSpeed(Robot.intakeLowerMotorSpeed.getDouble(0),
        Robot.intakeHigherMotorSpeed.getDouble(0), Robot.transportationMotorOneSpeed.getDouble(0),
        Robot.transportationMotorTwoSpeed.getDouble(0)));
  }

  public Command stopMotorsCommand() {
    return this.run(this::stopMotors);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
