// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;

public class ScoreMid extends CommandBase {
  /** Creates a new ScoreMid. */
  Tower m_tower;
  private double kp;
  private double adjustment;
  private double targetAngle;
  public ScoreMid(Tower tower) {
    addRequirements(tower);
    m_tower = tower;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
  //   adjustment = kp * Math.abs(targetAngle-m_tower.getCalculatedEncoderAngle());
  //   if(m_tower.getCalculatedEncoderAngle() <= targetAngle + 2)
  //   {
  //     m_tower.runSpeed(adjustment);
  //   }
  //   else if(m_tower.getCalculatedEncoderAngle() >= targetAngle - 2)
  //   {
  //     m_tower.runSpeed(-adjustment);
  //   }
  //   else
  //   {
  //     end(isFinished());
  //   }

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
