// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tower;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatchResult;

public class DefaultClawCommand extends CommandBase {
  /** Creates a new DefaultClawCommand. */
  Claw m_claw;
  XboxController joy2;

  public DefaultClawCommand(Claw claw, XboxController xbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_claw = claw;
    joy2 = xbox;
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(joy2.getRawAxis(2) > 0.2)
    {
        m_claw.pickItem();
    }
    else if (joy2.getRawAxis(3) > 0.2)
    {
      m_claw.dropItem();
    } else {
      m_claw.setClawSpeed(0);
    }
    if(joy2.getRawButton(5))
    {
      m_claw.m_clawDoubleSolenoid.toggle();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.setClawSpeed((0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
