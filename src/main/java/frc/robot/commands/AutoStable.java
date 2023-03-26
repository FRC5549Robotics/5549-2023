// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStable extends InstantCommand {

  DrivetrainSubsystem m_drivetrain;
  double angle, oldAngle;
  boolean endCommand;
  double steering_adjust;
  double kp = 0.04;
  PIDController controller = new PIDController(kp, 0, 0);


  public AutoStable(DrivetrainSubsystem drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    endCommand = false;
    angle = m_drivetrain.getCurrentPitch();

    // if(angle <= oldAngle)
    // {
    //   m_drivetrain.drive(new ChassisSpeeds(0,-0.4*steering_adjust,0));
    // }
    // else {
    //   m_drivetrain.drive(new ChassisSpeeds(0,0.4 * steering_adjust,0));
    // }
    if(angle < -3 || angle > 3){
      m_drivetrain.drive(new ChassisSpeeds(Math.sin(Math.toRadians(angle))*-1.45, 0, 0));
    }
    else{
      endCommand = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    System.out.println("Finished AutoStable");
    m_drivetrain.lockModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
  }
}
