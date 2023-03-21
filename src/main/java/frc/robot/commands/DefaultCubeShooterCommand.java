// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CubeShooter;
import frc.robot.subsystems.Tower;

public class DefaultCubeShooterCommand extends CommandBase {
  /** Creates a new DefaultCubeShooterCommand. */
  private XboxController m_controller;
  private CubeShooter m_CubeShooter;
  private 


  public DefaultCubeShooterCommand(CubeShooter cubeShooter, XboxController m_Controller, Tower tower) {
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
    if(m_controller.getRawButton(7))
    {
      m_CubeShooter.setSpeed(0.125);
      System.out.println("yes");
    }
    else if(m_controller.getRawButton(8))
    {
      System.out.println("yds");
      m_CubeShooter.setSpeed(-1);
    }
    else
    {
      m_CubeShooter.setSpeed(0);
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
