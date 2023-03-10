// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Claw;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.commands.ExtendMedium;
import frc.robot.commands.Retract;
import frc.robot.commands.RunIntakeAuto;
import frc.robot.commands.AutoAlignCommands.AutoAlign2;
import frc.robot.commands.Pivot;

import com.pathplanner.lib.PathPlannerTrajectory;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveConeAuto extends SequentialCommandGroup {
  /** Creates a new ThreeConeAuto. */
  DrivetrainSubsystem m_drivetrainSubsystem;
  Intake m_intake;
  Telescope m_telescope;
  Tower m_tower;
  Limelight m_limelight;
  Claw m_claw;
  XboxController rumController;
  PathPlannerTrajectory Path1;
  PathPlannerTrajectory Path2;
  PathPlannerTrajectory Path3;
  PathPlannerTrajectory Path4;
  PathPlannerTrajectory Path5;
  PathPlannerTrajectory Path6;
  PathPlannerTrajectory Path7;
  PathPlannerTrajectory Path8;
  public FiveConeAuto(DrivetrainSubsystem drivetrainSubsystem, Intake intake, Telescope telescope, Tower tower, Limelight limelight, Claw claw, XboxController RumController, PathPlannerTrajectory TopToCone,
  PathPlannerTrajectory TopBackToCone, PathPlannerTrajectory TopSecondConePickup, PathPlannerTrajectory TopSecondConeBack, PathPlannerTrajectory TopThirdConePickup,
  PathPlannerTrajectory TopThirdConeBack, PathPlannerTrajectory TopFourthConePickup, PathPlannerTrajectory TopFourthConeBack) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_intake = intake;
    m_telescope = telescope;
    m_tower = tower;
    m_limelight = limelight;
    m_claw = claw;
    rumController = RumController;
    Path1 = TopToCone;
    Path2 = TopBackToCone;
    Path3 = TopSecondConePickup;
    Path4 = TopSecondConeBack;
    Path5 = TopThirdConePickup;
    Path6 = TopThirdConeBack;
    Path7 = TopFourthConePickup;
    Path8 = TopFourthConeBack;
    addCommands(
      new InstantCommand(() -> {
          m_drivetrainSubsystem.resetOdometry(Path1.getInitialHolonomicPose());
      }),
      new ExtendMedium(m_telescope, rumController),
      new InstantCommand(m_claw::dropItem),
      new ParallelCommandGroup(
        new Retract(m_telescope),
        m_drivetrainSubsystem.followTrajectoryCommand(Path1),
        new RunIntakeAuto(m_intake)
      ),
      new ParallelCommandGroup(
        m_drivetrainSubsystem.followTrajectoryCommand(Path2),
        new ExtendMedium(m_telescope, rumController),
        new Pivot(m_tower)
      ),
      new AutoAlign2(m_limelight, m_drivetrainSubsystem),
      new InstantCommand(m_claw::dropItem),
      new ParallelCommandGroup(
        new Retract(m_telescope),
        m_drivetrainSubsystem.followTrajectoryCommand(Path3),
        new RunIntakeAuto(m_intake)
      ),
      new ParallelCommandGroup(
        m_drivetrainSubsystem.followTrajectoryCommand(Path4),
        new ExtendMedium(m_telescope, rumController),
        new Pivot(m_tower)
      ),
      new AutoAlign2(m_limelight, m_drivetrainSubsystem),
      new InstantCommand(m_claw::dropItem),
      new ParallelCommandGroup(
        new Retract(m_telescope),
        m_drivetrainSubsystem.followTrajectoryCommand(Path5),
        new RunIntakeAuto(m_intake)
      ),
      new ParallelCommandGroup(
        new ExtendMedium(m_telescope, rumController),
        m_drivetrainSubsystem.followTrajectoryCommand(Path6),
        new Pivot(m_tower)
      ),
      new AutoAlign2(m_limelight, m_drivetrainSubsystem),
      new InstantCommand(m_claw::dropItem),
      new ParallelCommandGroup(
        new Retract(m_telescope),
        m_drivetrainSubsystem.followTrajectoryCommand(Path7),
        new RunIntakeAuto(m_intake)
      ),
      new ParallelCommandGroup(
        new ExtendMedium(m_telescope, rumController),
        m_drivetrainSubsystem.followTrajectoryCommand(Path8),
        new Pivot(m_tower)
      ),
      new AutoAlign2(m_limelight, m_drivetrainSubsystem),
      new InstantCommand(m_claw::dropItem)
    );
  }
}
