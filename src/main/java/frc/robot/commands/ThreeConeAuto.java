// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.commands.ExtendMedium;
import frc.robot.commands.Retract;
import frc.robot.commands.RunIntakeAuto;
import frc.robot.commands.AutoAlign2;
import frc.robot.commands.Pivot;

import com.pathplanner.lib.PathPlannerTrajectory;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeConeAuto extends SequentialCommandGroup {
  /** Creates a new ThreeConeAuto. */
  DrivetrainSubsystem m_drivetrainSubsystem;
  Intake m_intake;
  Telescope m_telescope;
  Tower m_tower;
  Limelight m_limelight;
  PathPlannerTrajectory pathTopToCone;
  PathPlannerTrajectory pathTopBackToCone;
  PathPlannerTrajectory pathTopSecondConePickup;
  PathPlannerTrajectory pathTopSecondConeBack;
  public ThreeConeAuto(DrivetrainSubsystem drivetrainSubsystem, Intake intake, Telescope telescope, Tower tower, Limelight limelight, PathPlannerTrajectory TopToCone, 
  PathPlannerTrajectory TopBackToCone, PathPlannerTrajectory TopSecondConePickup, PathPlannerTrajectory TopSecondConeBack) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_intake = intake;
    m_telescope = telescope;
    m_tower = tower;
    m_limelight = limelight;
    pathTopToCone = TopToCone;
    pathTopBackToCone = TopBackToCone;
    pathTopSecondConePickup = TopSecondConePickup;
    pathTopSecondConeBack = TopSecondConeBack;
    addCommands(
      new InstantCommand(() ->{
          m_drivetrainSubsystem.GetInitialHeading();
      }),
      new InstantCommand(() -> {
          m_drivetrainSubsystem.resetOdometry(pathTopToCone.getInitialHolonomicPose());
      }),
      new ExtendMedium(m_telescope),
      new InstantCommand(m_tower::dropItem),
      new ParallelCommandGroup(
        new Retract(m_telescope),
        m_drivetrainSubsystem.followTrajectoryCommand(pathTopToCone),
        new RunIntakeAuto(m_intake)
      ),
      new ParallelCommandGroup(
        m_drivetrainSubsystem.followTrajectoryCommand(pathTopBackToCone),
        new ExtendMedium(m_telescope),
        new Pivot(m_tower)
      ),
      new AutoAlign2(m_limelight, m_drivetrainSubsystem),
      new InstantCommand(m_tower::dropItem),
      new ParallelCommandGroup(
        new Retract(m_telescope),
        m_drivetrainSubsystem.followTrajectoryCommand(pathTopSecondConePickup),
        new RunIntakeAuto(m_intake)
      ),
      new ParallelCommandGroup(
        m_drivetrainSubsystem.followTrajectoryCommand(pathTopSecondConeBack),
        new ExtendMedium(m_telescope),
        new Pivot(m_tower)
      ),
      new AutoAlign2(m_limelight, m_drivetrainSubsystem),
      new InstantCommand(m_tower::dropItem)
    );
  }
}