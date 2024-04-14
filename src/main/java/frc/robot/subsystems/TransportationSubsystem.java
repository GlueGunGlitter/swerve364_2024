// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class TransportationSubsystem extends SubsystemBase {

  // create two Mortors with spark flex
  CANSparkFlex intakeHigherMotor = new CANSparkFlex(Constants.TransportationConstants.INTAKE_HIGHER_MOTOR_PORT,
      MotorType.kBrushless);
  CANSparkFlex intakeLowerMotor = new CANSparkFlex(Constants.TransportationConstants.INTAKE_LOWER_MOTOR_PORT,
      MotorType.kBrushless);

  // create two motors with talon 
  WPI_TalonSRX transportationMotor1 = new WPI_TalonSRX(Constants.TransportationConstants.TRANSPORTATION_MOTOR_PORT_ONE);
  WPI_TalonSRX transportationMotor2 = new WPI_TalonSRX(Constants.TransportationConstants.TRANSPORTATION_MOTOR_PORT_TWO);

  // create sesor for the notes
  private final AnalogInput noteSensor = new AnalogInput(0);

  /** Creates a new IntakeSubsystem. */
  public TransportationSubsystem() {
    
    // set the invers mode
    intakeHigherMotor.setInverted(true);
    intakeLowerMotor.setInverted(true);
    transportationMotor1.setInverted(false);
    transportationMotor2.setInverted(false);

    // put elextisity limit
    transportationMotor1.configContinuousCurrentLimit(30);
    transportationMotor2.configContinuousCurrentLimit(30);
  }

  // set given speed to the motors 
  public void setSpeed(double intakeLowerMotorSpeed, double intakeHigherMotorSpeed, double transportationMotorOneSpeed,
      double transportationMotorTwoSpeed) {
    intakeHigherMotor.set(intakeHigherMotorSpeed);
    intakeLowerMotor.set(intakeLowerMotorSpeed);
    transportationMotor1.set(transportationMotorOneSpeed);
    transportationMotor2.set(transportationMotorTwoSpeed);

  }
  
  /*
   start intake and transportation with speed from the shafelbord 
   stop working when the sensor ditect note
  */
  private void upWithStop() {
    if (noteSensor.getValue() > 3000) {
      stopMotors();
    } else {
      setSpeed(Robot.intakeLowerMotorSpeed.getDouble(0),
          Robot.intakeHigherMotorSpeed.getDouble(0), Robot.transportationMotorOneSpeed.getDouble(0),
          Robot.transportationMotorTwoSpeed.getDouble(0));
    }
  }

  /* 
   start intake and transportation with speed from the shafelbord 
   dose not stop working when the sensor ditect note
  */
  private void upWithOutStop() {
    setSpeed(Robot.intakeLowerMotorSpeed.getDouble(0),
        Robot.intakeHigherMotorSpeed.getDouble(0), Robot.transportationMotorOneSpeed.getDouble(0),
        Robot.transportationMotorTwoSpeed.getDouble(0));

  }
  
  // stop the all the motors intak and tranportation
  public void stopMotors() {
    intakeHigherMotor.set(0);
    intakeLowerMotor.set(0);
    transportationMotor1.set(0);
    transportationMotor2.set(0);

  }
  
  /*
    command for the autonomus that start upwards transportation and intake 
    with given speed from the code and not from the shafelbord
  */
  public Command transportUpAutoCommand(double stopTimetransportUp) {
    return this.run(() -> setSpeed(0.8, 0.8, 0.7, 0.7)).withTimeout(stopTimetransportUp);
  }

  /*
    command for the autonomus that stop transportation and intake 
  */
  public Command stopTransportAutoCommand() {
    return this.run(this::stopMotors);
  }

  /*
    command for the autonomus that start backwards transportation and intake 
    with given speed from the code and not from the shafelbord
  */
  public Command transportDowmAutoCommand(double stopTimetransportDown) {
    return this.run(() -> setSpeed(0, 0, -0.5, -0.5))
    .withTimeout(stopTimetransportDown);
  }

  /*
    command for the telop that start backwards transportation and intake 
    with given speed from the shafelbord
  */
  public Command transportDownCommand() {
    return this.run(() -> setSpeed(Robot.intakeLowerMotorSpeed.getDouble(0) * -1,
        Robot.intakeHigherMotorSpeed.getDouble(0) * -1,
        Robot.transportationMotorOneSpeed.getDouble(0) * -1,
        Robot.transportationMotorTwoSpeed.getDouble(0) * -1));
  }

  /*
    command for the telep that start transpotation and intake
    stop whene the sensor detect note 
  */
  public Command transportUpWithStopCommand() {
    return this.run(() -> upWithStop());
  }

  /*
    command for the telep that start transpotation and intake
    dose not stop whene the sensor detect note 
  */
  public Command transportUpWithOutStopCommand() {
    return this.run(() -> upWithOutStop());
  }

  /*
    command for the autonmus that start upwards transpotation and intake
    stop whene the sensor detect note 
  */
  public Command transportUpWithStopAutoCommand(){
      return this.run(() -> upWithStop());
  }
  // command that stop the motors intake and transpotation
  public Command stopMotorsCommand() {
    return this.run(this::stopMotors);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
