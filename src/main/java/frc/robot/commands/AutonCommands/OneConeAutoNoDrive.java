// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Retract;
import frc.robot.commands.RunClawBackwards;
import frc.robot.commands.TelescopeStringPotAuton;
import frc.robot.commands.ExtendMedium;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CubeShooter;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Tower;
import frc.robot.commands.PivotEncoderAuton;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneConeAutoNoDrive extends SequentialCommandGroup {
  /** Creates a new OneConeAuto. */
  DrivetrainSubsystem m_DrivetrainSubsystem;
  Tower m_tower;
  Telescope m_telescope;
  Claw m_claw;
  CubeShooter CubeShooter;
  XboxController rumController;
  Tower.TargetLevel target1;
  public OneConeAutoNoDrive(DrivetrainSubsystem Drivetrain, Telescope telescope, Tower tower, Claw claw, CubeShooter CubeShooter, XboxController xbox, Tower.TargetLevel Target1) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_DrivetrainSubsystem = Drivetrain;
    m_tower = tower;
    m_telescope = telescope;
    m_claw = claw;
    rumController = xbox;
    this.CubeShooter = CubeShooter;
    target1 = Target1;
    addCommands(
      new ParallelCommandGroup(
      new PivotEncoderAuton(m_tower, Tower.TargetLevel.ConeHigh, m_claw, CubeShooter),
      //new ExtendMedium(m_telescope, rumController)
      new SequentialCommandGroup(
        new WaitCommand(1),
        new TelescopeStringPotAuton(Tower.TargetLevel.ConeHigh, telescope)
      )),
      new RunClawBackwards(m_claw, 1000.0),
      new ParallelCommandGroup(
        //new Retract(m_telescope),
        new TelescopeStringPotAuton(Tower.TargetLevel.Retracted, telescope),
        new PivotEncoderAuton(tower, Tower.TargetLevel.ConeMid, m_claw, CubeShooter)
      )
    );
  }
}
