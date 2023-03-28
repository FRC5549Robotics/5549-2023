// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CubeShooter extends SubsystemBase {

  public boolean canMove = false;
  Tower tower;
  /** Creates a new CubeShooter. */
  private CANSparkMax HingeMotor = new CANSparkMax(18, MotorType.kBrushless);
  private CANSparkMax ShooterMotor1 = new CANSparkMax(19, MotorType.kBrushless);
  private CANSparkMax ShooterMotor2 = new CANSparkMax(20, MotorType.kBrushless);
  RelativeEncoder HingeEncoder;
  RelativeEncoder ShooterMotor1Encoder;
  RelativeEncoder ShooterMotor2Encoder;

  public CubeShooter(Tower tower) {
    HingeEncoder = HingeMotor.getEncoder();
    ShooterMotor1Encoder = ShooterMotor1.getEncoder();
    ShooterMotor2Encoder = ShooterMotor2.getEncoder();
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

  public boolean getHingeMotorStatus(){
    if (HingeEncoder.getVelocity() > 0){
      return true;
    } else {
      return false;
    }
  }
  public boolean getLeftShooterMotorStatus(){
    if (ShooterMotor1Encoder.getVelocity() > 0){
      return true;
    } else {
      return false;
    }
  }

  public boolean getRightShooterMotorStatus(){
    if (ShooterMotor2Encoder.getVelocity() > 0){
      return true;
    } else {
      return false;
    }
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hinge Encoder Value:", GetEncoderValue());
    SmartDashboard.putBoolean("Hinge Motor Running?", getHingeMotorStatus());
    SmartDashboard.putBoolean("Left Motor Running?", getLeftShooterMotorStatus());
    SmartDashboard.putBoolean("Right Motor Running?", getRightShooterMotorStatus());
    // This method will be called once per scheduler run
  }
}
