// Copyright (c) FIRST and other WPILib contributors.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;


import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Retract;
import frc.robot.commands.WaitCommand;
import frc.robot.commands.AutoStable;
import frc.robot.commands.ExtendMedium;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CubeShooter;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Tower;
import frc.robot.commands.PivotEncoder;
import frc.robot.commands.PivotEncoderAuton;
import frc.robot.commands.AutoStable;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneConeChargeStation extends SequentialCommandGroup {
  /** Creates a new OneConeAuto. */
  DrivetrainSubsystem m_DrivetrainSubsystem;
  Tower m_tower;
  Telescope m_telescope;
  Claw m_claw;
  CubeShooter CubeShooter;
  XboxController rumController;
  PathPlannerTrajectory path1;
  Tower.TargetLevel target1;
  public OneConeChargeStation(DrivetrainSubsystem Drivetrain, Telescope telescope, Tower tower, Claw claw, CubeShooter CubeShooter, XboxController xbox, Tower.TargetLevel Target1,
  PathPlannerTrajectory path1) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_DrivetrainSubsystem = Drivetrain;
    m_tower = tower;
    m_telescope = telescope;
    m_claw = claw;
    this.CubeShooter = CubeShooter;
    rumController = xbox;
    target1 = Target1;
    this.path1 = path1;
    addCommands(
      new InstantCommand(() -> {
        m_DrivetrainSubsystem.resetOdometry(path1.getInitialHolonomicPose());
      }),
      new ParallelCommandGroup(
      new PivotEncoderAuton(m_tower, target1, m_claw, CubeShooter),
      new ExtendMedium(m_telescope, rumController)
      ),
      new WaitCommand(500),
      new ParallelCommandGroup(
        m_DrivetrainSubsystem.followTrajectoryCommand(path1),
        new Retract(m_telescope)
      ),
      new ParallelCommandGroup(
        new AutoStable(m_DrivetrainSubsystem),
        new PivotEncoder(tower, Tower.TargetLevel.ConeMid, claw, CubeShooter)
      )
    );
  }
}
