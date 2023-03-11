// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;

public class PivotTimed extends CommandBase {
  /** Creates a new PivotTimed. */
  double m_time;
  double startTime;
  Tower m_tower;
  boolean myAutoFinished = false;
  public PivotTimed(Tower tower) {
    m_tower = tower;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    m_time = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_time = (System.currentTimeMillis() - startTime) / 1000;
    if ((m_time >= 0.0) && (m_time < 5)){
      m_tower.runSpeed(0.5);
    }
    if (m_time >= 5){
      myAutoFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tower.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return myAutoFinished;
  }
}
