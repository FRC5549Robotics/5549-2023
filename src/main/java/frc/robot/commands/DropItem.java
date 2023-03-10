// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Telescope;

public class DropItem extends CommandBase {
  /** Creates a new DropItem. */
  Telescope telescope;
  public DropItem(Telescope telescope) {
    this.telescope = telescope;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(telescope);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    telescope.pickItem();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    telescope.stopClaw();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
