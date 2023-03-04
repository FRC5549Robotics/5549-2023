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


public class PositionTower extends CommandBase {
  /** Creates a new TowerTop. */
  Tower m_Tower;
  private final double TowerTopMeasure = 0.4;
  double kP, kI, kD;
  PIDController pid = new PIDController(kP, kI, kD);


  public PositionTower(Tower Top) {
    m_Tower = Top;
    addRequirements(m_Tower);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Tower.runSpeed(pid.calculate(m_Tower.GetEncoderValue(), TowerTopMeasure));
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
