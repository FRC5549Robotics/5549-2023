// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import com.revrobotics.AbsoluteEncoder;

public class Tower extends SubsystemBase {
  /** Creates a new Tower. */
  CANSparkMax motor1;
  CANSparkMax motor2;
  DoubleSolenoid m_clawDoubleSolenoid;

  // RelativeEncoder encoder1;
  // RelativeEncoder encoder2;
  private double MidLowPos = 40;
  private double calculatedEncoder;
  private AbsoluteEncoder throughBoreEncoder;
  public Tower() {
    motor1 = new CANSparkMax(Constants.MOTOR_TOWER1, MotorType.kBrushless);
    motor2 = new CANSparkMax(Constants.MOTOR_TOWER2, MotorType.kBrushless);
    AbsoluteEncoder throughBoreEncoder = motor1.getAbsoluteEncoder(Type.kDutyCycle);
    m_clawDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 6);
    // encoder1 = motor1.getAlternateEncoder(0);
    // encoder2 = motor2.getAlternateEncoder(0);
    
  }

  @Override
  public void periodic() {
    // calculatedEncoder = encoder1.getPosition()/64;
    // This method will be called once per scheduler run
  }

  public void Clamp(){
    m_clawDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void dropItem(){
    m_clawDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  public void runSpeed(double speed)
  {
    motor1.set(speed);
    motor2.set(speed);
  }

  public void runTo() {
    motor1.set(0.5);
    motor2.set(-0.5);
  }
  // public double getEncoderAngle()
  // {
  //   return encoder1.getPosition();
  // }
  // public double getCalculatedEncoderAngle()
  // {
  //   return encoder1.getPosition()/64;
  // }


  public void runBack() {
    motor1.set(-0.5);
    motor2.set(0.5);
  }

  public void off() {
    motor1.set(0);
    motor2.set(0);
  }
}
