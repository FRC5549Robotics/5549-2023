// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AutoAlignYaw extends CommandBase {
  /** Creates a new AutoAlignYaw. */
  Limelight m_Limelight;
  DrivetrainSubsystem m_drivetrain;
  boolean finished;
  PIDController controller;

  public AutoAlignYaw(Limelight limelight, DrivetrainSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Limelight = limelight;
    m_drivetrain = drivetrain;
    addRequirements(limelight);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnAngle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6])[5]; // IDK if yaw is index 5
    while (turnAngle > (10*Math.PI)/180 || turnAngle < -(10*Math.PI)/180)
      if (turnAngle > 0) {
        m_drivetrain.drive(new ChassisSpeeds(0, 0, -(8*Math.PI)/180));
      }
      else {
        m_drivetrain.drive(new ChassisSpeeds(0, 0, (8*Math.PI)/180));
      }
      turnAngle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6])[5]; // IDK if yaw is index 5
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
