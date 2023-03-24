// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Claw;
import frc.robot.Constants;

public class ExtendMedium extends CommandBase {
  /** Creates a new ExtendFar. */

  double startTime;
  Telescope m_Telescope;
  Claw m_Claw;
  boolean finished;
  XboxController rumController;
  public ExtendMedium(Telescope telescope, XboxController RumController, Claw claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Telescope = telescope;
    rumController = RumController;
    m_Claw = claw;
    addRequirements(telescope, claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    rumController.setRumble(RumbleType.kBothRumble, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finished = false;
    if (System.currentTimeMillis() - startTime < 1650) {
      m_Telescope.on(Constants.armSpeed);
    }
    else{
      finished = true;
    }  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Telescope.off();
    m_Claw.setConeMode();
    System.out.println("Finished telescope extension");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
