// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CubeShooter;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Tower.TargetLevel;

public class ShootCube extends CommandBase {
  /** Creates a new ShootCube. */
  double setpoint;
  CubeShooter m_CubeShooter;
  Tower.TargetLevel target;
  public ShootCube(CubeShooter cubeShooter, Tower.TargetLevel Target) {
    // Use addRequirements() here to declare subsystem dependencies.
    target = Target;
    m_CubeShooter = cubeShooter;
    addRequirements(cubeShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(target == Tower.TargetLevel.CubeHigh) setpoint = Constants.CUBE_SHOOTER_HIGH_SETPOINT;
    else if(target == Tower.TargetLevel.CubeMid) setpoint = Constants.CUBE_SHOOTER_MID_SETPOINT;
    else if(target == Tower.TargetLevel.CubeLow) setpoint = Constants.CUBE_SHOOTER_LOW_SETPOINT;
    else ;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_CubeShooter.RunShooter(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_CubeShooter.ShooterOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
