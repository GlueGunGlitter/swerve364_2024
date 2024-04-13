// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  // create two Mortors with spark flex
  CANSparkFlex nonStaticMotor = new CANSparkFlex(Constants.ShooterConstants.NON_STATIC_MOTOR_PORT,
      MotorType.kBrushless);
  CANSparkFlex staticMotor = new CANSparkFlex(Constants.ShooterConstants.STATIC_MOTOR_PORT,
      MotorType.kBrushless);

  // the speed for each motor
  public double nonStaticSpeed;
  public double staticMotorSpeed;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    // set inverted mode
    nonStaticMotor.setInverted(false);
    staticMotor.setInverted(true);

    // set the default speed to be 0
    staticMotorSpeed = 0;
    nonStaticSpeed = 0; 

  }

  // set the speed to the motors and shoot to th speeker
  public void shootUp(double nonStaticMotorSpeed, double staticMotorSpeed) {
    nonStaticMotor.set(-nonStaticMotorSpeed);
    staticMotor.set(staticMotorSpeed);

  }
  // set the speed to the motors and shoot to th AMP
  public void shootDown(double nonStaticMotorSpeed, double staticMotorSpeed) {
    nonStaticMotor.set(-nonStaticMotorSpeed);
    staticMotor.set(-staticMotorSpeed);

  }

  // set the speed to the motors and get the note back to the transporion
  public void shootBack(double nonStaticMotorSpeed, double staticMotorSpeed) {
    nonStaticMotor.set(nonStaticMotorSpeed);
    staticMotor.set(-staticMotorSpeed);
  }

    // stop the motors
  public void stopMotors() {
    nonStaticMotor.stopMotor();
    staticMotor.stopMotor();
  }

  /* 
  command for the autonomus that shoot to the speeker 
  with speed from the code and not from the shafellbord 
  */
  public Command shootUpAutoCommand(double stopTimeShootUp) {
    return this.run(() -> shootUp(0.8, 0.8)).withTimeout(stopTimeShootUp);
  }

   /* 
  command for the autonomus that shoot to the AMP
  with speed from the code and not from the shafellbord 
  */
  public Command shootDownAutoCommand(double stopTimeShootDown) {
    return this.run(() -> shootDown(0.4, 0.4)).withTimeout(stopTimeShootDown);
  }

   /* 
  command for the autonomus that get the note back the the taransportation
  with speed from the code and not from the shafellbord 
  */
  public Command shooterBackAutoCommand() {
    return this.run(() -> shootBack(0.1, 0.1)).withTimeout(0.1);
  }


   /*   
  command for the teleop that shoot to the speeker
  with speed from shafellbord 
  */
  public Command shootUpCommand() {
    return this.run(() -> shootUp(nonStaticSpeed, staticMotorSpeed))
        .withTimeout(ShooterConstants.SHOOT_TIMEOUT);
  }
   /* 
  command for the teleop that shoot to the APM
  with speed from shafellbord 
  */
  public Command shooterDownCommand() {
    return this.run(() -> shootDown(staticMotorSpeed - 0.3, nonStaticSpeed - 0.4))
        .withTimeout(ShooterConstants.SHOOT_TIMEOUT);
  }

 
  // stop the motors
  public Command stopMotorsCommand() {
    return this.run(this::stopMotors);
  }

  @Override
  public void periodic() {
    // set the speed to be the value from the shafelbord
    nonStaticSpeed = Robot.nonStaticShooterMotorSpeed.getDouble(0);
    staticMotorSpeed = Robot.staticShooterMotorSpeed.getDouble(0);
  }
}
