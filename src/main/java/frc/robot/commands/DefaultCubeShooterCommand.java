// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CubeShooter;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.UltrasonicSensor;

public class DefaultCubeShooterCommand extends CommandBase {
  /** Creates a new DefaultCubeShooterCommand. */
  private XboxController m_controller;
  private XboxController m_controller2;
  private CubeShooter m_CubeShooter;
  double TowerEncoderValue;
  double HingeEncoderValue;
  UltrasonicSensor m_Sensor;
  PIDController controller = new PIDController(0.02, 0, 0);

  public DefaultCubeShooterCommand(CubeShooter cubeShooter, XboxController m_Controller, XboxController m_Controller2, UltrasonicSensor sensor) {
    m_controller = m_Controller;
    m_controller2 = m_Controller2;
    m_CubeShooter = cubeShooter;
    m_Sensor = sensor;
    addRequirements(cubeShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TowerEncoderValue = m_CubeShooter.getTowerEncoderValue();
    HingeEncoderValue = m_CubeShooter.GetEncoderValue();
    SmartDashboard.putNumber("POV Controller: ", m_controller2.getPOV());
    //Intake the cube speed: -0.125

    //Outtake speed High
    if (m_Sensor.getSensorValue() < 10) {
      m_CubeShooter.setSpeed(0);
    }
    else if(m_controller2.getPOV() == 0)
    {
      m_CubeShooter.RunHinge(controller.calculate(HingeEncoderValue, Constants.CUBE_HINGE_HIGH_SETPOINT));
      if (m_controller2.getRawAxis(3) >0.1){
        m_CubeShooter.setSpeed(0.3); //0.3
      } else {  
        m_CubeShooter.setSpeed(0);
      }
    }
    else if (m_controller2.getPOV() == 90)
    {
      m_CubeShooter.RunHinge(controller.calculate(HingeEncoderValue, Constants.CUBE_HINGE_RETRACTED_SETPOINT));
      m_CubeShooter.setSpeed(0);
    }

    //Outtake speed Mid.
    else if(m_controller2.getPOV() == 270)
    {
      m_CubeShooter.RunHinge(controller.calculate(HingeEncoderValue, Constants.CUBE_HINGE_HIGH_SETPOINT));
      if (m_controller2.getRawAxis(3) > 0.1){
        m_CubeShooter.setSpeed(0.13);
      } else {
        m_CubeShooter.setSpeed(0);
      }
    }

    //Outtake speed Low
    else if (m_controller2.getPOV() == 180)
    {
      m_CubeShooter.RunHinge(controller.calculate(HingeEncoderValue, Constants.CUBE_HINGE_LOW_AND_INTAKE_SETPOINT));
      if (m_controller2.getRawAxis(3) > 0.1){
        m_CubeShooter.setSpeed(0.2);
      } else if (m_controller2.getRawAxis(2)> 0.1){
        m_CubeShooter.setSpeed(-0.13);
      } else{
        m_CubeShooter.setSpeed(0);
      }
    }

    //Shooter Up
    else if(m_controller.getRawButton(6))
    { 
      m_CubeShooter.setSpeed(0);
      m_CubeShooter.RunHinge(0.2);
    }

    //Shooter Down
    else if(m_controller.getRawButton(5))
    {
      m_CubeShooter.setSpeed(0);
      m_CubeShooter.RunHinge(-0.2);
    }

    //Shooter Speed
    else if (m_controller.getRawAxis(3) > 0.1){
      m_CubeShooter.setSpeed(m_controller.getRawAxis(3));
      m_CubeShooter.RunHinge(controller.calculate(HingeEncoderValue, HingeEncoderValue));
    } 
    

    else {
      m_CubeShooter.setSpeed(0);
      if (TowerEncoderValue < -0.2){
           m_CubeShooter.RunHinge(controller.calculate(HingeEncoderValue, Constants.CUBE_HINGE_RETRACTED_SETPOINT));
          } else {
           m_CubeShooter.RunHinge(controller.calculate(HingeEncoderValue, 29.5));
         }
      //m_CubeShooter.HingeOff();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_CubeShooter.HingeOff();
    m_CubeShooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
