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
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatchResult;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  CANSparkMax ClawMotor;
  public DoubleSolenoid m_clawDoubleSolenoid;

  AddressableLED LED;
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);
  Color kGreen1 = new Color(0,255, 0);
  Color kPurple1 = new Color(255,19,180);
  Color kYellow1 = new Color(255, 230, 0);
  public Claw(AddressableLED led){

  ClawMotor = new CANSparkMax(17, MotorType.kBrushless);
  ClawMotor.setIdleMode(IdleMode.kBrake);
  m_clawDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 3);

  LED = led;
  LED.setLength(ledBuffer.getLength());
  LED.setData(ledBuffer);
  LED.start();
  for(int i = 0; i < ledBuffer.getLength(); i++){
    ledBuffer.setLED(i, kGreen1);
  }
  LED.setData(ledBuffer);
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
  
  public void LEDToggle(){
    if(Constants.yellow){
      System.out.println("Color Detected");
      for(int i = 0; i<ledBuffer.getLength();i++)
      {
        ledBuffer.setLED(i, kPurple1);
      }
      LED.setData(ledBuffer);
    }
    else{
      for(int i = 0; i<ledBuffer.getLength();i++)
      {
        ledBuffer.setLED(i, kYellow1);
      }
      LED.setData(ledBuffer);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
