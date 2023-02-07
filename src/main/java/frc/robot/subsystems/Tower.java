// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class Tower extends SubsystemBase {
  /** Creates a new Tower. */
  CANSparkMax motor1;
  CANSparkMax motor2;
  public Tower() {
    motor1 = new CANSparkMax(69, MotorType.kBrushless);
    motor2 = new CANSparkMax(420, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run() {
    motor1.set(0.5);
    motor2.set(-0.5);
  }

  public void off() {
    motor1.set(0);
    motor2.set(0);
  }
}
