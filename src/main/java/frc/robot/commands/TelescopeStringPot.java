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

public class TelescopeStringPot extends CommandBase {
  /** Creates a new PivotEncoder. */
  Tower.TargetLevel state;
  boolean finished;
  PIDController cubeController = new PIDController(0.05, 0, 0);
  Telescope m_telescope;

  double setpoint;
  double stringPotValue;
  public TelescopeStringPot(Tower.TargetLevel State, Telescope telescope) {
    // Use addRequirements() here to declare subsystem dependencies.
    state = State;
    m_telescope = telescope;
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
    else if(state == Tower.TargetLevel.PickUpFront)setpoint = Constants.EXTEND_PICKUP_FRONT;
    else setpoint = Constants.PIVOT_RETRACTED_SETPOINT;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    HingeEncoderValue = cubeShooter.GetEncoderValue();
    finished = false;
    double currentAngle = m_Tower.GetEncoderValue();
    System.out.println(setpoint);
    System.out.println(currentAngle);
    if (setpoint == Constants.PIVOT_RETRACTED_SETPOINT){
      cubeShooter.RunHinge(cubeController.calculate(HingeEncoderValue, 29.5));
    } else {
      cubeShooter.RunHinge(cubeController.calculate(HingeEncoderValue, 0.0));
    }

    if ( currentAngle - setpoint > 0.01 || currentAngle - setpoint < -0.01){
    m_Tower.runSpeed(controller.calculate(currentAngle, setpoint));
  }
    else{
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("finished");
    m_Tower.off();
    m_claw.stopClaw();
    cubeShooter.HingeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
