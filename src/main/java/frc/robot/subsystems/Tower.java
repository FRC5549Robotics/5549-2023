// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;


import frc.robot.Constants;

public class Tower extends SubsystemBase {
  /** Creates a new Tower. */
  CANSparkMax motor1;
  CANSparkMax motor2;

  public Encoder throughBoreEncoder;
  public RelativeEncoder Encoder1;
  public Tower() {
    motor1 = new CANSparkMax(Constants.MOTOR_TOWER1, MotorType.kBrushless);
    motor2 = new CANSparkMax(Constants.MOTOR_TOWER2, MotorType.kBrushless);
    throughBoreEncoder = new Encoder(0, 1);
    Encoder1 = motor1.getEncoder();
    // encoder2 = motor2.getAlternateEncoder(0);
    
  }

  @Override
  public void periodic() {
    // calculatedEncoder = encoder1.getPosition()/64;
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Through Bore Encoder Value:", throughBoreEncoder.getDistance());
    SmartDashboard.putNumber("Motor 1 Encoder Values:", Encoder1.getPosition());
  }

  public boolean Pivot(PIDController controller, double currentAngle, double setpoint){
    boolean finished = false;
    if (currentAngle - setpoint > 2 || currentAngle - setpoint < -2){
    runSpeed(controller.calculate(currentAngle, setpoint));
    }
    else{
      finished = true;
    }
    return finished;
  }
  
  public void runSpeed(double speed)
  {
    motor1.set(speed);
    motor2.set(-speed);
  }

  public void runTo() {
    motor1.set(0.5);
    motor2.set(-0.5);
  }
  public double GetEncoderValue()
  {
    return throughBoreEncoder.getDistance();
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
