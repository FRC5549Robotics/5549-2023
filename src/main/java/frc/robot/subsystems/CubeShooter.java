// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CubeShooter extends SubsystemBase {

  private XboxController m_Controller;
  /** Creates a new CubeShooter. */
  private CANSparkMax HingeMotor = new CANSparkMax(31, MotorType.kBrushless);
  private TalonSRX ShooterMotor1 = new TalonSRX(34);
  private TalonSRX ShooterMotor2 = new TalonSRX(35);

  public CubeShooter(XboxController Controller) {
    m_Controller = Controller;

  }

  public void setSpeed(double speed)
  {
    ShooterMotor1.set(TalonSRXControlMode.PercentOutput, speed);
    ShooterMotor2.set(TalonSRXControlMode.PercentOutput, -speed);
  }
  public void setHighPosition(){
    HingeMotor.set(1);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
