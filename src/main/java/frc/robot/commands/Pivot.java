// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;


public class Pivot extends CommandBase {
  /** Creates a new Pivot. */
  Tower m_Tower;
  double startTime;
  boolean finished;
  public Pivot(Tower Tower) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Tower = Tower;
    addRequirements(Tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (System.currentTimeMillis() - startTime < 5000) {
      m_Tower.runTo();
    }
    finished = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Tower.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
