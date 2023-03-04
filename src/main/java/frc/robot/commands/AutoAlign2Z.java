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
import edu.wpi.first.wpilibj.XboxController;


public class AutoAlign2Z extends CommandBase {
  /** Creates a new AutoAlign2. */
  XboxController xbox1;
  Limelight m_Limelight;
  DrivetrainSubsystem m_drivetrain;

  double heading;
  PIDController controller2 = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  boolean finished;
  

  public AutoAlign2Z(Limelight Limelight, DrivetrainSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Limelight = Limelight;
    m_drivetrain = drivetrain;
    controller2.enableContinuousInput(-180, 180);
    addRequirements(Limelight);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double current_heading = m_drivetrain.m_navx.getAngle();
    System.out.println(Constants.INITIAL_HEADING);
    System.out.println(current_heading);
    System.out.println(current_heading - Constants.INITIAL_HEADING);
    if(current_heading - Constants.INITIAL_HEADING > 3 || current_heading - Constants.INITIAL_HEADING < -3){
      if (current_heading < 0){
        System.out.println("Yo its trying to run");
        m_drivetrain.drive(new ChassisSpeeds(0, 0, -controller2.calculate(current_heading, Constants.INITIAL_HEADING)));
      } else {
        m_drivetrain.drive(new ChassisSpeeds(0, 0, controller2.calculate(current_heading, Constants.INITIAL_HEADING)));
      }
    }
    else{
      finished = true;
    }
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
