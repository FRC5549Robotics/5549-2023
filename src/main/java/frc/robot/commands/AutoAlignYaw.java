// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;


public class AutoAlignYaw extends CommandBase {
  /** Creates a new AutoAlignYaw. */
  Limelight m_Limelight;
  DrivetrainSubsystem m_drivetrain;
  boolean finished = false;
  PIDController controller;
  double kP = 0.1;
  XboxController xbox;


  public AutoAlignYaw(Limelight limelight, DrivetrainSubsystem drivetrain, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Limelight = limelight;
    m_drivetrain = drivetrain;
    xbox = controller;
    addRequirements(limelight);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnAngle = m_Limelight.getYaw();
    if (turnAngle > (2*Math.PI)/180 || turnAngle < -(2*Math.PI)/180){
      if (turnAngle > 0) {
        m_drivetrain.drive(new ChassisSpeeds(0, 0, (Math.toRadians(turnAngle))));
      }
      else {
        m_drivetrain.drive(new ChassisSpeeds(0, 0, -(Math.toRadians(turnAngle))));
      }
  } else{
    finished = true;
  }
}


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.drive(new ChassisSpeeds(0,0,0));
    xbox.setRumble(RumbleType.kBothRumble, 1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
