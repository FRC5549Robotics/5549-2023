// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatchResult;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax motor_intake_1;
  DoubleSolenoid mDoubleSolenoidIntakeRight;
  DoubleSolenoid mDoubleSolenoidIntakeLeft;
  ColorSensorV3 m_colorSensor;
  I2C.Port i2cPort;
  public ColorMatch m_colorMatcher;
  public Color kYellowTarget = new Color(0.281982, 0.501465, 0.216064);
  public Color kPurpleTarget = new Color(0.230957,0.379883,0.395996);
  public Color kAir = new Color(0.254639, 0.487549, 0.258057);
  Color detectedColor;
  ColorMatchResult match;
  public Intake() {
    motor_intake_1 = new CANSparkMax(Constants.MOTOR_INTAKE_1, MotorType.kBrushed);
    mDoubleSolenoidIntakeRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    mDoubleSolenoidIntakeLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    i2cPort = I2C.Port.kOnboard;
    m_colorSensor = new ColorSensorV3(i2cPort);
    m_colorMatcher = new ColorMatch();
    m_colorMatcher.addColorMatch(kYellowTarget);
    m_colorMatcher.addColorMatch(kPurpleTarget);
    m_colorMatcher.addColorMatch(kAir);
  }

  public void run_intake(){
    motor_intake_1.set(0.5);
  }

  public void stop_intake(){
    motor_intake_1.set(0);
  }

  public void intake_out(){
    mDoubleSolenoidIntakeRight.set(DoubleSolenoid.Value.kForward);
    mDoubleSolenoidIntakeLeft.set(DoubleSolenoid.Value.kForward);
  }

  public void retract_intake(){
    mDoubleSolenoidIntakeLeft.set(DoubleSolenoid.Value.kReverse);
    mDoubleSolenoidIntakeRight.set(DoubleSolenoid.Value.kReverse);
  }

  public Color getColor(){
    return detectedColor;
  }
  public boolean color_check(){
    match = m_colorMatcher.matchClosestColor(detectedColor);
    if (match.color == kAir){
      return false;
    } else {
      return true;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    detectedColor = m_colorSensor.getColor();
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
  }
}
