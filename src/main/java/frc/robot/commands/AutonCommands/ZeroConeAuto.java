// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroConeAuto extends SequentialCommandGroup {
  /** Creates a new ZeroConeAuto. */
  DrivetrainSubsystem m_drivetrainSubsystem;
  PathPlannerTrajectory path;
  public ZeroConeAuto(DrivetrainSubsystem drivetrainSubsystem, PathPlannerTrajectory path) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_drivetrainSubsystem = drivetrainSubsystem;
    this.path = path;
    addCommands(
      new InstantCommand(() -> {
            m_drivetrainSubsystem.resetOdometry(path.getInitialHolonomicPose());
         }),
           m_drivetrainSubsystem.followTrajectoryCommand(path)
    );
  }
}
