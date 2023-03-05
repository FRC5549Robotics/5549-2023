// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

import java.lang.constant.DirectMethodHandleDesc;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;


public class AutoAlign2Z extends CommandBase {
  /** Creates a new AutoAlign2. */
  XboxController xbox1;
  Limelight m_Limelight;
  DrivetrainSubsystem m_drivetrain;

  double heading;
  PIDController controller2 = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  boolean finished;
  

  public AutoAlign2Z(Limelight Limelight, DrivetrainSubsystem drivetrain, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Limelight = Limelight;
    m_drivetrain = drivetrain;
    xbox1 = controller;
    controller2.enableContinuousInput(-360, 360);
    addRequirements(Limelight, drivetrain);
  }

  // Called when the command is initially schedule
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double current_heading = m_drivetrain.m_navx.getAngle()%360;
    System.out.println(Constants.INITIAL_HEADING);
    System.out.println(current_heading);
    System.out.println(current_heading - Constants.INITIAL_HEADING);
    if(current_heading - Constants.INITIAL_HEADING > 3 || current_heading - Constants.INITIAL_HEADING < -3){
      if (current_heading < 0){
        System.out.println("Yo its trying to run and angle is negative");
        System.out.println("Error its trying to use:" + controller2.calculate(heading, Constants.INITIAL_HEADING));
        m_drivetrain.drive(new ChassisSpeeds(0, 0, -controller2.calculate(current_heading, Constants.INITIAL_HEADING)));
      } else {
        System.out.println("Yo its trying to run and angle is positive");
        System.out.println("Error its trying to use:" + controller2.calculate(heading, Constants.INITIAL_HEADING));
        m_drivetrain.drive(new ChassisSpeeds(0, 0, -controller2.calculate(current_heading, Constants.INITIAL_HEADING)));
      }
    }else{
      finished = true;
    }
  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    xbox1.setRumble(RumbleType.kBothRumble, 1);
    m_drivetrain.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
