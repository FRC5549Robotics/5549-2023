// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetVision;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class PipelineAutoAlign extends CommandBase {
  /** Creates a new PipelineAutoAlign. */
  Limelight m_Limelight;
  TargetVision target;
  DrivetrainSubsystem m_DrivetrainSubsystem;
  PIDController controller = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  public PipelineAutoAlign(Limelight limelight, DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Limelight = limelight;
    m_DrivetrainSubsystem = drivetrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    target = m_Limelight.getTarget();
    if (target == TargetVision.Cube) {
      double tx = m_Limelight.limelightCube.getEntry("tx").getDouble(0);
      m_DrivetrainSubsystem.drive(new ChassisSpeeds(0, controller.calculate(0, tx), 0));

    }
    if (target == TargetVision.Cone) {
      double tx = m_Limelight.limelightCone.getEntry("tx").getDouble(0);
      m_DrivetrainSubsystem.drive(new ChassisSpeeds(0, controller.calculate(0, tx), 0));

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
