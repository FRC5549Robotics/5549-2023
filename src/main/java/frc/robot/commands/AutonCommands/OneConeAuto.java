// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;


import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PivotTimed;
import frc.robot.commands.Retract;
import frc.robot.commands.ExtendFar;
import frc.robot.commands.ExtendMedium;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Tower;
import frc.robot.RobotContainer;
import frc.robot.commands.PivotEncoder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneConeAuto extends SequentialCommandGroup {
  /** Creates a new OneConeAuto. */
  DrivetrainSubsystem m_DrivetrainSubsystem;
  Tower m_tower;
  Telescope m_telescope;
  Claw m_claw;
  XboxController rumController;
  PathPlannerTrajectory path1;
  Tower.TargetLevel target1;
  public OneConeAuto(DrivetrainSubsystem Drivetrain, Telescope telescope, Tower tower, Claw claw, XboxController xbox, Tower.TargetLevel Target1,
  PathPlannerTrajectory path1) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_DrivetrainSubsystem = Drivetrain;
    m_tower = tower;
    m_telescope = telescope;
    m_claw = claw;
    rumController = xbox;
    target1 = Target1;
    this.path1 = path1;
    addCommands(
      new InstantCommand(() -> {
        m_DrivetrainSubsystem.resetOdometry(path1.getInitialHolonomicPose());
      }),
      new ParallelCommandGroup(
       // new ExtendMedium(m_telescope, rumController, m_claw),
        new PivotEncoder(m_tower, target1, m_claw)
      ),
      new PivotTimed(m_tower),
      new ExtendMedium(m_telescope, rumController, m_claw),
      new InstantCommand(m_claw::dropItem).withTimeout(1),
      new ParallelCommandGroup(
        m_DrivetrainSubsystem.followTrajectoryCommand(path1),
        new InstantCommand(m_claw::stopClaw),
        new Retract(m_telescope)
      )
    );
  }
}
