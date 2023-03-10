// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.XboxController;

public class DefaultIntakeCommand extends CommandBase {
  /** Creates a new RunIntake. */
  Intake m_intake;
  XboxController m_joy;
  public DefaultIntakeCommand(Intake intake, XboxController joy) {
    m_intake = intake;
    m_joy = joy;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_intake.intake_out();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //cone
    if(m_joy.getRawAxis(3) > 0.2)
    {
      //m_intake.intake_out();
      m_intake.run_intake_speed(-1);
    }
    
    //cube
    else if(m_joy.getRawAxis(2) > 0.2)
    {
      //m_intake.intake_out();
      m_intake.run_intake_speed(-0.75);
    }
    else
    {
      m_intake.stop_intake();
    }
    if(m_joy.getRawButton(5))
    {
      System.out.println("Button Pressed");
      m_intake.retract_intake();
    }
    if(m_joy.getRawButton(6))
    {
      System.out.println("Button Pressed");
      m_intake.intake_out();
    }
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_intake.retract_intake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
