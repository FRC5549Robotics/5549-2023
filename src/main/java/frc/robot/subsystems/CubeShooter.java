// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CubeShooter extends SubsystemBase {

  private XboxController m_Controller;
  public boolean canMove = false;
  Tower tower;
  /** Creates a new CubeShooter. */
  private CANSparkMax HingeMotor = new CANSparkMax(18, MotorType.kBrushless);
  private CANSparkMax ShooterMotor1 = new CANSparkMax(19, MotorType.kBrushless);
  private CANSparkMax ShooterMotor2 = new CANSparkMax(20, MotorType.kBrushless);
  RelativeEncoder HingeEncoder;

  public CubeShooter(XboxController Controller, Tower tower) {
    m_Controller = Controller;
    HingeEncoder = HingeMotor.getEncoder();
    this.tower = tower;

  }

  public double getTowerEncoderValue(){
    return tower.towerEncoderValue;
  }

  public void setSpeed(double speed)
  {
    ShooterMotor1.set(speed);
    ShooterMotor2.set(speed);
  }
  public void setHighPosition(){
    //HingeMotor.set(1);
  }
  public void setMidPosition(){
    //HingeMotor.set(0.5);
  }
  public void setLowPosition()
  {
    //HingeMotor.set(0);
  }
  public void RunHinge(double speed)
  {
    HingeMotor.set(speed);
  }
  public void HingeOff()
  {
    HingeMotor.set(0);
  }
  public void RunShooter(double speed)
  {
    ShooterMotor1.set(speed);
    ShooterMotor2.set(-speed);
  }
  public void ShooterOff(){
    ShooterMotor1.set(0);
    ShooterMotor2.set(0);
  }
  public double GetEncoderValue()
  {
    return HingeEncoder.getPosition();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hinge Encoder Value:", GetEncoderValue());
    SmartDashboard.putNumber("Tower Encoder Value", getTowerEncoderValue());
    // This method will be called once per scheduler run
  }
}
