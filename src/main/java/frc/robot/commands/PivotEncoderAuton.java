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


public class PivotEncoderAuton extends CommandBase {
  /** Creates a new PivotEncoder. */
  Tower.TargetLevel state;
  // 1 = in, 2 = mid, 3 = in

  Tower m_Tower;
  boolean finished;
  PIDController controller = new PIDController(2.6, 0, 0);
  PIDController cubeController = new PIDController(0.02, 0, 0);

  double setpoint;
  Claw m_claw;
  CubeShooter cubeShooter;
  double HingeEncoderValue;
  public PivotEncoderAuton(Tower Tower, Tower.TargetLevel State, Claw claw, CubeShooter cubeShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    state = State;
    m_Tower = Tower;
    m_claw = claw;
    this.cubeShooter = cubeShooter;
    addRequirements(Tower, claw, cubeShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(state == Tower.TargetLevel.ConeHigh)setpoint = Constants.PIVOT_CONE_HIGH_SETPOINT;
    else if(state == Tower.TargetLevel.Intake)setpoint = Constants.PIVOT_INTAKE_SETPOINT;
    else if(state == Tower.TargetLevel.ConeMid)setpoint = Constants.PIVOT_CONE_MID_SETPOINT;
    else if(state == Tower.TargetLevel.CubeHigh)setpoint = Constants.PIVOT_CUBE_HIGH_SETPOINT;
    else if(state == Tower.TargetLevel.CubeMid)setpoint = Constants.PIVOT_CUBE_MID_SETPOINT;
    else if(state == Tower.TargetLevel.PickUpFront)setpoint = Constants.CONE_PICKUP_FRONT;
    else setpoint = Constants.PIVOT_RETRACTED_SETPOINT;

    if (setpoint != Constants.PIVOT_RETRACTED_SETPOINT){
      //m_claw.runSlow();
    } else {
      m_claw.setCubeMode();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    HingeEncoderValue = cubeShooter.GetEncoderValue();
    finished = false;
    double currentAngle = m_Tower.GetEncoderValue();
    if (setpoint == Constants.PIVOT_CONE_MID_SETPOINT){
      cubeShooter.RunHinge(cubeController.calculate(HingeEncoderValue, 29.5));
      cubeShooter.setSpeed(-0.25);
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
    m_claw.setConeMode();
    cubeShooter.HingeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
