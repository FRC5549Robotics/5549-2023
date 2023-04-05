// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CubeShooter;
import frc.robot.subsystems.Tower;

public class ShootCube extends CommandBase {
  /** Creates a new ShootCube. */
  double setpoint;
  CubeShooter m_CubeShooter;
  Tower.TargetLevel target;
  PIDController controller;
  double HingeEncoderValue;
  boolean finished;
  public ShootCube(CubeShooter cubeShooter, Tower.TargetLevel Target) {
    // Use addRequirements() here to declare subsystem dependencies.
    target = Target;
    m_CubeShooter = cubeShooter;
    controller = new PIDController(0.2, 0, 0);
    addRequirements(cubeShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    if(target == Tower.TargetLevel.CubeHigh) setpoint = Constants.CUBE_HINGE_HIGH_SETPOINT;
    else if(target == Tower.TargetLevel.CubeMid) setpoint = Constants.CUBE_SHOOTER_MID_SETPOINT;
    else if(target == Tower.TargetLevel.CubeLow) setpoint = Constants.CUBE_SHOOTER_LOW_SETPOINT;
    else if(target == Tower.TargetLevel.Intake) setpoint = Constants.CUBE_SHOOTER_INTAKE_SETPOINT;
    else if(target == Tower.TargetLevel.Retracted) setpoint = 15;
    else;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    HingeEncoderValue = m_CubeShooter.GetEncoderValue();
    m_CubeShooter.RunHinge(controller.calculate(HingeEncoderValue, setpoint));
    if (Math.abs(HingeEncoderValue - setpoint) < 0.5){
      if (setpoint == 15){
        finished = true;
      } else {
        m_CubeShooter.setSpeed(0.3);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_CubeShooter.HingeOff();
    m_CubeShooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
