// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tower;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatchResult;

public class DefaultClawCommand extends CommandBase {
  /** Creates a new DefaultClawCommand. */
  Claw m_claw;
  Intake m_intake;
  XboxController joy2;
  Color detectedColor;
  ColorMatchResult match;
  boolean yellow = true;
  AddressableLED LED;
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);
  Color kGreen1 = new Color(0,255, 0);
  Color kPurple1 = new Color(255,19,180);
  Color kYellow1 = new Color(255, 255, 0);

  public DefaultClawCommand(Claw claw, Intake intake, XboxController xbox, AddressableLED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
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
    detectedColor = m_intake.getColor();
    match = m_intake.m_colorMatcher.matchClosestColor(detectedColor);

    if(joy2.getRawAxis(2) > 0.2)
    {
        m_claw.setClawSpeed(joy2.getRawAxis(2)*0.25);
    }
    else if (joy2.getRawAxis(3) > 0.2)
    {
      m_claw.setClawSpeed(-joy2.getRawAxis(3)*0.5);
    } else {
      m_claw.stopClaw();
    }

    if(joy2.getRawButton(5) || match.color == m_intake.kPurpleTarget)
    {
      m_claw.setCubeMode();
    }
    if(joy2.getRawButton(6) || match.color == m_intake.kYellowTarget)
    {
      m_claw.setConeMode();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.stopClaw();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
