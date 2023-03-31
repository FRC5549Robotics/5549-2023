// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlignCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;


import edu.wpi.first.wpilibj.XboxController;

public class AutoAlign extends CommandBase {
  
  double Kp = 1/27;
  double ty, tv, tx, angle, distance, ta, oldta;
  XboxController xbox1;
  double steering_adjust = 0.0;
  Limelight m_Limelight;
  DrivetrainSubsystem m_drivetrain;
  boolean endCommand = false;

  /** Creates a new AutoAlign. */
  public AutoAlign(Limelight Limelight, DrivetrainSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    //m_Limelight = Limelight;
    m_drivetrain = drivetrain;
    addRequirements(Limelight);
    //addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ta = m_Limelight.getTa();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
  }
}
