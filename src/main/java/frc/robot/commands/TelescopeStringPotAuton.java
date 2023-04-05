// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CubeShooter;
import frc.robot.subsystems.Telescope;
import frc.robot.Constants;

public class TelescopeStringPotAuton extends CommandBase {
  /** Creates a new PivotEncoder. */
  Tower.TargetLevel state;
  boolean finished;
  Telescope m_telescope;

  double setpoint;
  double stringPotValue;
  PIDController controller;
  public TelescopeStringPotAuton(Tower.TargetLevel State, Telescope telescope) {
    // Use addRequirements() here to declare subsystem dependencies.
    state = State;
    m_telescope = telescope;
    controller = new PIDController(20, 0, 0);
    addRequirements(m_telescope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(state == Tower.TargetLevel.ConeHigh)setpoint = Constants.EXTEND_CONE_HIGH_SETPOINT;
    else if(state == Tower.TargetLevel.Intake)setpoint = Constants.EXTEND_INTAKE_SETPOINT;
    else if(state == Tower.TargetLevel.ConeMid)setpoint = Constants.EXTEND_CONE_MID_SETPOINT;
    else if(state == Tower.TargetLevel.CubeHigh)setpoint = Constants.EXTEND_CUBE_HIGH_SETPOINT;
    else if(state == Tower.TargetLevel.CubeMid)setpoint = Constants.EXTEND_CUBE_MID_SETPOINT;
    else if(state == Tower.TargetLevel.Chute)setpoint = Constants.EXTEND_PICKUP_FRONT;
    else setpoint = Constants.EXTEND_RETRACTED;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finished = false;
      System.out.println("Trying to run");
      if (m_telescope.getStringPot() > setpoint + 0.001 || m_telescope.getStringPot() < setpoint - 0.001){
        m_telescope.on(-controller.calculate(m_telescope.getStringPot(), setpoint));
      } else{
        finished = true;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_telescope.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
