// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  CANSparkMax ClawMotor;
  public DoubleSolenoid m_clawDoubleSolenoid;


  public Claw() {

  ClawMotor = new CANSparkMax(17, MotorType.kBrushless);
  ClawMotor.setIdleMode(IdleMode.kBrake);
  m_clawDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);


  }
  public void pickItem()
  {
    ClawMotor.set(0.75);
  }

  public void dropItem(){
    ClawMotor.set(-0.15);
  }
  public void setClawSpeed(Double speed)
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

  public void stopClaw(){
    ClawMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
