// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tower;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatchResult;

public class DefaultClawCommand extends CommandBase {
  /** Creates a new DefaultClawCommand. */
  Intake m_intake;
  Tower m_tower;
  Color detectedColor;
  ColorMatchResult match;

  public DefaultClawCommand(Intake intake, Tower tower) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_tower = tower;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    detectedColor = m_intake.getColor();
    match = m_intake.m_colorMatcher.matchClosestColor(detectedColor);
    if (match.color == m_intake.kPurpleTarget){
      m_tower.Clamp();
    } else if (match.color == m_intake.kYellowTarget){
      m_tower.dropItem();
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
