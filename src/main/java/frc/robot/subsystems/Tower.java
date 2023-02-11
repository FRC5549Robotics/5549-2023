// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class Tower extends SubsystemBase {
  /** Creates a new Tower. */
  CANSparkMax motor1;
  CANSparkMax motor2;
  DoubleSolenoid m_clawDoubleSolenoid;
  public Tower() {
    motor1 = new CANSparkMax(Constants.MOTOR_TOWER1, MotorType.kBrushless);
    motor2 = new CANSparkMax(Constants.MOTOR_TOWER2, MotorType.kBrushless);
    m_clawDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 6);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Clamp(){
    m_clawDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void dropItem(){
    m_clawDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
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
