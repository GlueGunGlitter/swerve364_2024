// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkFlex nonStaticMotor = new CANSparkFlex(Constants.ShooterConstants.NON_STATIC_MOTOR_PORT,
      MotorType.kBrushless);
  CANSparkFlex staticMotor = new CANSparkFlex(Constants.ShooterConstants.STATIC_MOTOR_PORT,
      MotorType.kBrushless);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    nonStaticMotor.setInverted(true);
    staticMotor.setInverted(true);
  }

  public void shootUp(double nonStaticMotorSpeed, double staticMotorSpeed) {
    nonStaticMotor.set(nonStaticMotorSpeed);
    staticMotor.set(staticMotorSpeed);
  }

  public void shootDown(double nonStaticMotorSpeed, double staticMotorSpeed) {
    nonStaticMotor.set(-nonStaticMotorSpeed);
    staticMotor.set(staticMotorSpeed);
  }

  public void stopMotors() {
    nonStaticMotor.stopMotor();
    staticMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
