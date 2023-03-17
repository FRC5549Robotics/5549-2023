// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Tower;
import frc.robot.Constants;

public class ExtendTimed extends CommandBase {
  /** Creates a new ExtendTimed. */
  double startTime;
  Telescope m_Telescope;
  boolean finished = false;
  XboxController rumController;
  double setpoint;
  Tower.TargetLevel target;
  public ExtendTimed(Telescope telescope, XboxController RumController, Tower.TargetLevel Target) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Telescope = telescope;
    rumController = RumController;
    addRequirements(telescope);
    target = Target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    rumController.setRumble(RumbleType.kBothRumble, 0);
    if(target == Tower.TargetLevel.High)setpoint = Constants.EXTEND_HIGH_SETPOINT;
    else if(target == Tower.TargetLevel.Mid)setpoint = Constants.EXTEND_MID_SETPOINT;
    else finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Telescope.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
