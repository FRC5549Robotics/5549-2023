// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CubeShooter;
import edu.wpi.first.math.controller.PIDController;

public class DefaultCubeShooterCommand extends CommandBase {
  /** Creates a new DefaultCubeShooterCommand. */
  private XboxController m_controller;
  private CubeShooter m_CubeShooter;
  double TowerEncoderValue;
  double HingeEncoderValue;
  PIDController controller = new PIDController(0.01, 0, 0);

  public DefaultCubeShooterCommand(CubeShooter cubeShooter, XboxController m_Controller) {
    m_controller = m_Controller;
    m_CubeShooter = cubeShooter;
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
    if(m_controller.getRawButton(5))
    {
      m_CubeShooter.setSpeed(-0.125);
      System.out.println("yes");
    }
    else if(m_controller.getRawButton(6))
    {
      m_CubeShooter.setSpeed(1);
    }
    else
    {
      m_CubeShooter.setSpeed(0);
    }
    if(m_controller.getRawAxis(2) > 0.1)
    { 
      m_CubeShooter.RunHinge(m_controller.getRawAxis(2)/5);
    }
    else if(m_controller.getRawAxis(3) > 0.1)
    {
      m_CubeShooter.RunHinge(-m_controller.getRawAxis(3)/5);
    } else {
       if (TowerEncoderValue < -0.2 && m_CubeShooter.canMove){
        m_CubeShooter.RunHinge(controller.calculate(HingeEncoderValue, 15.7));
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
