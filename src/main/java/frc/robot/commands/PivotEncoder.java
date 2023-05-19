// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CubeShooter;


public class PivotEncoder extends CommandBase {
  /** Creates a new PivotEncoder. */
  Tower.TargetLevel state;
  // 1 = in, 2 = mid, 3 = in

  Tower m_Tower;
  boolean finished;
  PIDController controller = new PIDController(2.9, 0, 0.1);
  PIDController controllerR = new PIDController(2.3, 0, 0);
  PIDController cubeController = new PIDController(0.01, 0, 0);

  double setpoint;
  CubeShooter cubeShooter;
  double HingeEncoderValue;
  public PivotEncoder(Tower Tower, Tower.TargetLevel State, CubeShooter cubeShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    state = State;
    m_Tower = Tower;
    this.cubeShooter = cubeShooter;
    addRequirements(Tower, cubeShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(state == Tower.TargetLevel.ConeHigh)setpoint = Constants.PIVOT_CONE_HIGH_SETPOINT;
    else if(state == Tower.TargetLevel.Intake)setpoint = Constants.PIVOT_INTAKE_SETPOINT;
    else if(state == Tower.TargetLevel.ConeMid)setpoint = Constants.PIVOT_CONE_MID_SETPOINT;
    else if(state == Tower.TargetLevel.CubeHigh)setpoint = Constants.PIVOT_CUBE_HIGH_SETPOINT;
    else if(state == Tower.TargetLevel.CubeMid)setpoint = Constants.PIVOT_CUBE_MID_SETPOINT;
    else if(state == Tower.TargetLevel.Chute)setpoint = Constants.PIVOT_CONE_CHUTE_SETPOINT;
    else if(state == Tower.TargetLevel.Substation)setpoint = Constants.PIVOT_CONE_SUBSTATION;
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
      if (setpoint == Constants.PIVOT_RETRACTED_SETPOINT){
        m_Tower.runSpeed(controllerR.calculate(currentAngle, setpoint));
      }else {
        m_Tower.runSpeed(controller.calculate(currentAngle, setpoint));
        }
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
    cubeShooter.HingeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
