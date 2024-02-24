// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.fasterxml.jackson.core.json.DupDetector;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private WPI_TalonSRX intakeLowerMotor;
    private WPI_VictorSPX intakeHigherMotor;
    private PIDController lowPIDController;
    private PIDController highPIDController;

    public Boolean state = false; 


  /** Creates a new Intake. */
  public IntakeSubsystem() {
    intakeLowerMotor = new WPI_TalonSRX(Constants.TransportationConstants.INTAKE_LOWER_MOTOR_PORT);
    intakeHigherMotor = new WPI_VictorSPX(Constants.TransportationConstants.INTAKE_HIGHER_MOTOR_PORT);
    lowPIDController = new PIDController(0.1, 0, 0); // Update with your PID constants
    highPIDController = new PIDController(0.1, 0, 0);

    intakeLowerMotor.configFactoryDefault();   
    intakeLowerMotor.configFactoryDefault();


  }

  public void setSpeed(double lowerSpeed,double highSpeed){
    lowPIDController.setSetpoint(lowerSpeed);
    highPIDController.setSetpoint(highSpeed);
    
    state = true;
  }

  public void cancel(){
    lowPIDController.setSetpoint(0);
    highPIDController.setSetpoint(0);

    state = false;

  }
  private double convertVelocityToRPM(double velocityBeforConvert){
    double velocityInRPM = velocityBeforConvert*600;  
    return velocityInRPM;
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double lowOutput = lowPIDController.calculate(convertVelocityToRPM(intakeLowerMotor.getSelectedSensorVelocity()));
    double highOutput = highPIDController.calculate(convertVelocityToRPM(intakeHigherMotor.getSelectedSensorVelocity()));

    intakeLowerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, lowOutput);
    intakeHigherMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, highOutput);


  }
}
