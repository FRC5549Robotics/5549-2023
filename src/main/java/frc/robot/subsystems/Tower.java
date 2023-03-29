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
import edu.wpi.first.wpilibj.DutyCycleEncoder;


import frc.robot.Constants;

public class Tower extends SubsystemBase {
  /** Creates a new Tower. */
  public enum TargetLevel{
    Retracted,
    Intake,
    CubeLow,
    CubeMid,
    CubeHigh,
    ConeMid,
    ConeHigh,
    ShooterAim,
    PickUpFront
  }
  CANSparkMax motor1;
  CANSparkMax motor2;

  
  public DutyCycleEncoder throughBoreEncoder;
  public RelativeEncoder Encoder1, Encoder2;
  public double towerEncoderValue;
  public Tower() {
    motor1 = new CANSparkMax(Constants.MOTOR_TOWER1, MotorType.kBrushless);
    motor2 = new CANSparkMax(Constants.MOTOR_TOWER2, MotorType.kBrushless);
    throughBoreEncoder = new DutyCycleEncoder(0);
    throughBoreEncoder.setPositionOffset(0.674);
    Encoder1 = motor1.getEncoder();
    Encoder2 = motor2.getEncoder();
    // encoder2 = motor2.getAlternateEncoder(0);
    
  }

  @Override
  public void periodic() {
    // calculatedEncoder = encoder1.getPosition()/64;
    // This method will be called once per scheduler run
    towerEncoderValue = throughBoreEncoder.getDistance();
    SmartDashboard.putNumber("Through Bore Encoder Value:", throughBoreEncoder.getDistance());
    SmartDashboard.putNumber("Motor 1 Encoder Values:", Encoder1.getPosition());
    SmartDashboard.putBoolean("Tower Motor 1 Running?", getTowerMotor1Status());
    SmartDashboard.putBoolean("Tower Motor 2 Running?", getTowerMotor2Status());
  }

  public boolean getTowerMotor1Status(){
    if (Encoder1.getVelocity() > 0){
      return true;
    } else {
      return false;
    }
  }

  public boolean getTowerMotor2Status(){
    if (Encoder2.getVelocity() > 0){
      return true;
    } else {
      return false;
    }
  }

  public boolean Pivot(PIDController controller, double currentAngle, double setpoint){

    if (currentAngle - setpoint > 2 || currentAngle - setpoint < -2){
    runSpeed(controller.calculate(currentAngle, setpoint));
    }
    else{
      return true;
    }
    return false;
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


  public void runBack() {
    motor1.set(-0.5);
    motor2.set(0.5);
  }

  public void off() {
    motor1.set(0);
    motor2.set(0);
  }
}
