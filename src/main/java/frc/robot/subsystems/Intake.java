// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax motor_intake_1;
  DoubleSolenoid mDoubleSolenoid1;
  DoubleSolenoid mDoubleSolenoid2;
  public Intake() {
    motor_intake_1 = new CANSparkMax(Constants.MOTOR_INTAKE_1, MotorType.kBrushed);
    mDoubleSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
    mDoubleSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);
  }

  public void run_intake(){
    motor_intake_1.set(0.5);
  }

  public void intake_out(){
    mDoubleSolenoid1.set(DoubleSolenoid.Value.kForward);
    mDoubleSolenoid2.set(DoubleSolenoid.Value.kForward);
  }

  public void retract_intake(){
    mDoubleSolenoid1.set(DoubleSolenoid.Value.kReverse);
    mDoubleSolenoid2.set(DoubleSolenoid.Value.)
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
