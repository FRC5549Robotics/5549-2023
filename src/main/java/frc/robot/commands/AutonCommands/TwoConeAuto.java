// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ExtendFar;
import frc.robot.commands.ExtendMedium;
import frc.robot.commands.PivotMid;
import frc.robot.commands.PivotHigh;
import frc.robot.commands.Retract;
import frc.robot.commands.RunIntakeAuto;
import frc.robot.commands.AutoAlignCommands.AutoAlign2;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Claw;
import frc.robot.RobotContainer;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoConeAuto extends SequentialCommandGroup {
  /** Creates a new RunAuto. */
  DrivetrainSubsystem m_drivetrainSubsystem;
  Intake m_intake;
  Claw m_claw;
  Telescope m_telescope;
  Tower m_tower;
  Limelight m_limelight;
  XboxController rumController;
  public TwoConeAuto(DrivetrainSubsystem drivetrainSubsystem, Intake intake, Telescope telescope, Tower tower, Limelight limelight, Claw claw, XboxController RumController) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_telescope = telescope;
    m_tower = tower;
    m_limelight = limelight;
    m_claw = claw;
    m_intake = intake;
    rumController = RumController;
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> {
          m_drivetrainSubsystem.resetOdometry(RobotContainer.TopToCT1.getInitialHolonomicPose());
      }),
      new ExtendMedium(m_telescope, rumController),
      new InstantCommand(m_claw::dropItem),
      new Retract(m_telescope),
      m_drivetrainSubsystem.followTrajectoryCommand(RobotContainer.TopToCT1),
      new RunIntakeAuto(m_intake),
      new ParallelCommandGroup(
        m_drivetrainSubsystem.followTrajectoryCommand(RobotContainer.CT1ToTop),
        new ExtendMedium(m_telescope, rumController),
        new PivotMid(m_tower)
      ),
      new AutoAlign2(m_limelight, m_drivetrainSubsystem),
      new InstantCommand(m_claw::dropItem)
    );
  }
}
