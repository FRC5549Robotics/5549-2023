// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CubeShooter;
import frc.robot.subsystems.Tower;

public class ShooterAim extends CommandBase {
  boolean finished = false;
  Tower.TargetLevel state;
  double setpoint;
  PIDController controller = new PIDController(0.05, 0, 0);
  CubeShooter m_CubeShooter;
  XboxController xbox;
  /** Creates a new ShooterAim. */
  public ShooterAim(CubeShooter CubeShooter, Tower.TargetLevel State, XboxController xbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    state = State;
    m_CubeShooter = CubeShooter;
    this.xbox = xbox;
    addRequirements(CubeShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(state == Tower.TargetLevel.ShooterAim) setpoint = Constants.CUBE_HINGE_HIGH_SETPOINT;
    else if(state == Tower.TargetLevel.CubeMid) setpoint = Constants.CUBE_HINGE_MID_SETPOINT;
    else if((state == Tower.TargetLevel.CubeLow || state == Tower.TargetLevel.Intake) && m_CubeShooter.canMove) setpoint = Constants.CUBE_HINGE_LOW_AND_INTAKE_SETPOINT;
    else if(state == Tower.TargetLevel.Retracted) setpoint = Constants.CUBE_HINGE_RETRACTED_SETPOINT;
    else finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finished = false;
    double currentAngle = m_CubeShooter.GetEncoderValue();
    System.out.println(setpoint);
    System.out.println(currentAngle);
    if ( currentAngle - setpoint > 0.01 || currentAngle - setpoint < -0.01){
    m_CubeShooter.RunHinge(controller.calculate(currentAngle, setpoint));
  }
    else{
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_CubeShooter.HingeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
