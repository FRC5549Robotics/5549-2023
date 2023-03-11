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
import edu.wpi.first.wpilibj.XboxController;


public class AutoAlign2X extends CommandBase {
  /** Creates a new AutoAlign2. */
  XboxController xbox1;
  Limelight m_Limelight;
  DrivetrainSubsystem m_drivetrain;

  double heading;
  PIDController turnController;
  PIDController controller2;
  boolean finished;
  

  public AutoAlign2X(Limelight Limelight, DrivetrainSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Limelight = Limelight;
    m_drivetrain = drivetrain;
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
    finished = false;
    System.out.println(m_Limelight.getTx());
    controller2 = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    if(m_Limelight.getTx() > 3 || m_Limelight.getTx()<-3){
      System.out.println("trying to drive");
      System.out.println("Error:" + controller2.calculate(m_Limelight.getTx(), 0));
      m_drivetrain.drive(new ChassisSpeeds(0, controller2.calculate(m_Limelight.getTx(), 9.5), 0));
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
