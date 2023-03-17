// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Tower;


public class PivotEncoder extends CommandBase {
  /** Creates a new PivotEncoder. */
  Tower.TargetLevel state;
  // 1 = in, 2 = mid, 3 = in

  Tower m_Tower;
  boolean finished = false;
  PIDController controller = new PIDController(0.5, 0, 0);
  double setpoint;
  public PivotEncoder(Tower Tower, Tower.TargetLevel State) {
    // Use addRequirements() here to declare subsystem dependencies.
    state = State;
    m_Tower = Tower;
    addRequirements(Tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(state == Tower.TargetLevel.High)setpoint = Constants.PIVOT_HIGH_SETPOINT;
    else if(state == Tower.TargetLevel.Intake)setpoint = Constants.PIVOT_INTAKE_SETPOINT;
    else if(state == Tower.TargetLevel.Mid)setpoint = Constants.PIVOT_MID_SETPOINT;
    else setpoint = Constants.PIVOT_RETRACTED_SETPOINT;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = m_Tower.GetEncoderValue();
    // if ( currentAngle - setpoint > 2 || currentAngle - setpoint < -2){
    // m_Tower.runSpeed(controller.calculate(currentAngle, setpoint));
    // }
    // else{
    //   finished = true;
    // }
    finished = m_Tower.Pivot(controller, currentAngle, setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Tower.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
