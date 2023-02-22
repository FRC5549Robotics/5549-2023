// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.nio.file.Path;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AutoAlign2;
import frc.robot.commands.ExtendFar;
import frc.robot.commands.ExtendMedium;
import frc.robot.commands.Pivot;
import frc.robot.commands.PivotBack;
import frc.robot.commands.Retract;
import frc.robot.commands.RunIntakeAuto;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Limelight;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoConeAuto extends SequentialCommandGroup {
  /** Creates a new RunAuto. */
  DrivetrainSubsystem m_drivetrainSubsystem;
  Intake m_intake;
  PathPlannerTrajectory Path1;
  PathPlannerTrajectory Path2;
  Telescope m_telescope;
  Tower m_tower;
  Limelight m_limelight;
  public TwoConeAuto(DrivetrainSubsystem drivetrainSubsystem, Intake intake, Telescope telescope, Tower tower, Limelight limelight, PathPlannerTrajectory path1, 
  PathPlannerTrajectory path2) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_telescope = telescope;
    m_tower = tower;
    m_limelight = limelight;
    Path1 = path1;
    Path2 = path2;
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() ->{
          m_drivetrainSubsystem.GetInitialHeading();
      }),
        new InstantCommand(() -> {
          m_drivetrainSubsystem.resetOdometry(Path1.getInitialHolonomicPose());
      }),
      new ExtendMedium(m_telescope),
      new InstantCommand(m_tower::dropItem),
      new Retract(m_telescope),
      m_drivetrainSubsystem.followTrajectoryCommand(Path1),
      new RunIntakeAuto(m_intake),
      new ParallelCommandGroup(
        m_drivetrainSubsystem.followTrajectoryCommand(Path2),
        new ExtendMedium(m_telescope),
        new Pivot(m_tower)
      ),
      new AutoAlign2(m_limelight, m_drivetrainSubsystem),
      new InstantCommand(m_tower::dropItem)
    );
  }
}
