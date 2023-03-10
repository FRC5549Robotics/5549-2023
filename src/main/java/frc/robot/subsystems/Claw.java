// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  CANSparkMax ClawMotor;
  public DoubleSolenoid m_clawDoubleSolenoid;


  public Claw() {




  }
  public void setClawSpeed(double speed)
  {
    ClawMotor.set(speed);
  }
  public void setConeMode()
  {
    m_clawDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  public void setCubeMode()
  {
    m_clawDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
