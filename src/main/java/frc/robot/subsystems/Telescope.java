// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


public class Telescope extends SubsystemBase {
  /** Creates a new Telescope. */
  CANSparkMax TelescopeMotor;
  CANSparkMax ClawMotor;
  PIDController pid = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  XboxController rumController;
  double startTime;
  Telescope m_Telescope;
  RelativeEncoder telescopeMEncoder;
  boolean finished = false;
  AnalogPotentiometer stringPot = new AnalogPotentiometer(2,1,0);

  public Telescope(XboxController controller) {
    TelescopeMotor = new CANSparkMax(Constants.MOTOR_TELESCOPE_1, MotorType.kBrushless);
    telescopeMEncoder = TelescopeMotor.getEncoder();
    rumController = controller;
  }

  public boolean getTelescopeMotorStatus(){
    if (telescopeMEncoder.getVelocity() > 0){
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Telescope Motor Running?", getTelescopeMotorStatus());
    SmartDashboard.putNumber("String Pot", stringPot.get());
  }

  public double getEncoder()
  {
    return TelescopeMotor.getEncoder().getPosition();
  }

  public boolean RunToTarget(double startTime, double timeSetpoint){
    if (System.currentTimeMillis() - startTime < timeSetpoint) {
      m_Telescope.on(Constants.armSpeed);
    }
    else{
      return true;
    }
    return false;
  }

  public void on(double speed) {
    TelescopeMotor.set(speed);
  }
  public void off() {
    TelescopeMotor.set(0);
  }

  public double getStringPot() {
    return stringPot.get();
  }
}
