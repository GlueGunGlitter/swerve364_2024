// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class TureToAngelTest extends SubsystemBase {
  PIDController pidController = new PIDController(0.033, 0, 0);

  /** Creates a new TureToAngelTest. */
  public TureToAngelTest() {

  }

  public double rotationVal() {
    return pidController.calculate(-30, 0);
  }

  @Override
  public void periodic() {
    System.out.println(rotationVal());
  }
}
