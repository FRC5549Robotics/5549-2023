// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Telescope;
import edu.wpi.first.wpilibj.XboxController;

public class DefaultTelescopeCommand extends CommandBase {
  /** Creates a new TelescopeCommand. */
  private Telescope m_tele;
  private XboxController m_joy2;
  public DefaultTelescopeCommand(Telescope tele, XboxController joy2) {
    m_tele = tele;
    m_joy2 = joy2;
    addRequirements(tele);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    if(Math.abs(m_joy2.getRawAxis(5)) > 0.1)
    {
      m_tele.on(m_joy2.getRawAxis(5));
    }
    else
    {
      m_tele.off();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tele.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
