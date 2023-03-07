// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;


public class Telescope extends SubsystemBase {
  /** Creates a new Telescope. */
  CANSparkMax TelescopeMotor;
  CANSparkMax ClawMotor;
  PIDController pid = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    

  public Telescope() {
    TelescopeMotor = new CANSparkMax(Constants.MOTOR_TELESCOPE_1, MotorType.kBrushless);
    ClawMotor = new CANSparkMax(Constants.MOTOR_CLAW_1, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public double getEncoder()
  {
    return TelescopeMotor.getEncoder().getPosition();
  }

  public void on(double speed) {
    TelescopeMotor.set(speed);
  }

  public void dropItem(){
    ClawMotor.set(-0.5);
  }

  public void off() {
    TelescopeMotor.set(0);
  }
}
