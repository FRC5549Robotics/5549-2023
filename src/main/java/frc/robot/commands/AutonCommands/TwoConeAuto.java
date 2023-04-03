// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ExtendMedium;
import frc.robot.commands.PivotEncoderAuton;
import frc.robot.commands.Retract;
import frc.robot.commands.ShootCube;
import frc.robot.commands.WaitCommand;
import frc.robot.commands.AutoAlignCommands.AutoAlign2;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CubeShooter;
import frc.robot.RobotContainer;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoConeAuto extends SequentialCommandGroup {
  /** Creates a new RunAuto. */
  DrivetrainSubsystem m_drivetrainSubsystem;
  Claw m_claw;
  Telescope m_telescope;
  Tower m_tower;
  Limelight m_limelight;
  CubeShooter CubeShooter;
  XboxController rumController;
  PathPlannerTrajectory trajectory;
  PathPlannerTrajectory trajectory2;
  public TwoConeAuto(DrivetrainSubsystem drivetrainSubsystem, Telescope telescope, Tower tower, Limelight limelight, Claw claw, CubeShooter CubeShooter, XboxController RumController, PathPlannerTrajectory Trajectory, PathPlannerTrajectory Trajectory2) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_telescope = telescope;
    m_tower = tower;
    m_limelight = limelight;
    m_claw = claw;
    this.CubeShooter = CubeShooter;
    rumController = RumController;
    trajectory = Trajectory;
    trajectory2 = Trajectory2;
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> {
          m_drivetrainSubsystem.resetOdometry(trajectory.getInitialHolonomicPose());
      }),
      new ParallelCommandGroup(
        new PivotEncoderAuton(m_tower, Tower.TargetLevel.ConeHigh, m_claw, CubeShooter),
        new ExtendMedium(m_telescope, RumController)
      ),
      new WaitCommand(500),
      new ParallelCommandGroup(
        new PivotEncoderAuton(m_tower, Tower.TargetLevel.Retracted, m_claw, CubeShooter),
        new Retract(m_telescope),
        m_drivetrainSubsystem.followTrajectoryCommand(trajectory)
      ),
      new ParallelCommandGroup(
        new ShootCube(CubeShooter, Tower.TargetLevel.Retracted),
        new SequentialCommandGroup(
          m_drivetrainSubsystem.followTrajectoryCommand(trajectory2),
          new AutoAlign2(m_limelight, m_drivetrainSubsystem)
        )
      ),
        new ShootCube(CubeShooter, Tower.TargetLevel.CubeHigh)
      //new InstantCommand(m_claw::dropItem)
    );
  }
}
