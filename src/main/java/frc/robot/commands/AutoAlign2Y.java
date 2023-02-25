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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;


public class AutoAlign2Y extends CommandBase {
  /** Creates a new AutoAlign2. */
  XboxController rumController;
  Limelight m_Limelight;
  DrivetrainSubsystem m_drivetrain;

  double heading;
  PIDController turnController;
  PIDController controller2;
  boolean finished;
  

  public AutoAlign2Y(Limelight Limelight, DrivetrainSubsystem drivetrain, XboxController RumController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Limelight = Limelight;
    m_drivetrain = drivetrain;
    rumController = RumController;
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
    turnController = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    controller2 = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    //m_drivetrain.drive(new ChassisSpeeds(0, controller2.calculate((calDistance()/12)*0.3048)-Constants.POLE_DISTANCE, 0));
    m_drivetrain.drive(new ChassisSpeeds(0, controller2.calculate(m_Limelight.calDistance(), Constants.POLE_DISTANCE),0));
    if(m_Limelight.calDistance() - Constants.POLE_DISTANCE < 3){
      finished = true;
    }
  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    rumController.setRumble(RumbleType.kBothRumble, 1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
