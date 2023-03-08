// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.MotorCommutation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;


public class DefaultTowerCommand extends CommandBase {
  /** Creates a new TowerTop. */
  Tower m_Tower;
  private final double TowerTopMeasure = 0.6;
  private final double TowerGrabMeause = 0.22;
  private final double TowerLowerMeasure = 0.3;
  private final double TowerMidMeasure = 0.28;
  private String m_Location;
  double kP, kI, kD;
  PIDController pid = new PIDController(kP, kI, kD);
  XboxController m_joy;

  public DefaultTowerCommand(Tower Top, XboxController joy) {
    m_Tower = Top;
    m_joy = joy;
    addRequirements(m_Tower);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Math.abs(m_joy.getRawAxis(1)) > 0.1)
    {
      m_Tower.runSpeed(m_joy.getRawAxis(1));
    }
    else if(m_joy.getBButton())
    {
      m_Tower.runSpeed(pid.calculate(m_Tower.GetEncoderValue(), TowerTopMeasure));
    }
    else if(m_joy.getYButton())
    {
      m_Tower.runSpeed(pid.calculate(m_Tower.GetEncoderValue(), TowerMidMeasure));
    }
    else if(m_joy.getXButton())
    {
      m_Tower.runSpeed(pid.calculate(m_Tower.GetEncoderValue(), TowerLowerMeasure));
    }
    else if(m_joy.getAButton())
    {
      m_Tower.runSpeed(pid.calculate(m_Tower.GetEncoderValue(), TowerGrabMeause));
    } else{
      m_Tower.off();
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
