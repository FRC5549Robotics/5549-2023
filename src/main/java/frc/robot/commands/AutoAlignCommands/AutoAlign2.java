// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlignCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.controller.PIDController;


public class AutoAlign2 extends CommandBase {
  /** Creates a new AutoAlign2. */
  Limelight m_Limelight;
  DrivetrainSubsystem m_drivetrain;

  double heading;
  PIDController turnController;
  PIDController controller2;
  boolean finished;
  

  public AutoAlign2(Limelight Limelight, DrivetrainSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Limelight = Limelight;
    m_drivetrain = drivetrain;
    addRequirements(Limelight, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double current_heading = m_drivetrain.m_navx.getAngle();
    System.out.println(current_heading);
    System.out.println(m_Limelight.getTx());
    turnController = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    controller2 = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    m_drivetrain.drive(new ChassisSpeeds(0, 0, turnController.calculate((current_heading*Math.PI)/180, (Constants.INITIAL_HEADING*Math.PI)/180)));
    m_drivetrain.drive(new ChassisSpeeds(controller2.calculate(m_Limelight.getTx(), 0), 0, 0));
    m_drivetrain.drive(new ChassisSpeeds(0, ((m_Limelight.calDistance()/12)*0.3048)-Constants.POLE_DISTANCE, 0));
    finished = true;
  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
