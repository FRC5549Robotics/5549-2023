// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class UltrasonicSensor extends SubsystemBase {
  /** Creates a new UltrasonicSensor. */
  AnalogPotentiometer sensor = new AnalogPotentiometer(0, 500, 500);
  public UltrasonicSensor() {}

  public double getSensorValue() {
    return sensor.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
