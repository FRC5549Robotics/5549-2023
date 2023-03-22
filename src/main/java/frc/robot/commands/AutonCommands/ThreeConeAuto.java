// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import frc.robot.RobotContainer;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CubeShooter;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.commands.ExtendMedium;
import frc.robot.commands.PivotEncoder;
import frc.robot.commands.Retract;
import frc.robot.commands.RunIntakeAuto;
import frc.robot.commands.AutoAlignCommands.AutoAlign2;


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
  Claw m_claw;
  CubeShooter m_CubeShooter;
  XboxController rumController;
  Tower.TargetLevel target1;
  Tower.TargetLevel target2;
  Tower.TargetLevel target3;
  public ThreeConeAuto(DrivetrainSubsystem drivetrainSubsystem, Intake intake, Telescope telescope, Tower tower, Limelight limelight, Claw claw, CubeShooter Cubeshooter, XboxController RumController, Tower.TargetLevel Target1, Tower.TargetLevel Target2, Tower.TargetLevel Target3) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_intake = intake;
    m_telescope = telescope;
    m_tower = tower;
    m_limelight = limelight;
    m_claw = claw;
    rumController = RumController;
    target1 = Target1;
    target2 = Target2;
    target2 = Target3;
    m_CubeShooter = Cubeshooter;
    
    addCommands(
      new InstantCommand(() -> {
          m_drivetrainSubsystem.resetOdometry(RobotContainer.TopToCT1.getInitialHolonomicPose());
      }),
      new ParallelCommandGroup(
        m_drivetrainSubsystem.followTrajectoryCommand(RobotContainer.MidTToCT2),
        //new ExtendMedium(m_telescope, rumController, m_claw),
        new PivotEncoder(m_tower, Target1, m_claw, m_CubeShooter)
      ),
      new ExtendMedium(m_telescope, rumController, m_claw),
      new InstantCommand(m_claw::dropItem),
      new ParallelCommandGroup(
        new Retract(m_telescope),
        m_drivetrainSubsystem.followTrajectoryCommand(RobotContainer.CT1ToMidT),
        new RunIntakeAuto(m_intake)
      ),
      new ParallelCommandGroup(
        m_drivetrainSubsystem.followTrajectoryCommand(RobotContainer.MidTToCT2),
        //new ExtendMedium(m_telescope, rumController, m_claw),
        new PivotEncoder(m_tower, Target2, m_claw, m_CubeShooter)
      ),
      new AutoAlign2(m_limelight, m_drivetrainSubsystem),
      new InstantCommand(m_claw::dropItem),
      new ParallelCommandGroup(
        new Retract(m_telescope),
        m_drivetrainSubsystem.followTrajectoryCommand(RobotContainer.CT2ToMidT),
        new RunIntakeAuto(m_intake)
      ),
      new ParallelCommandGroup(
        m_drivetrainSubsystem.followTrajectoryCommand(RobotContainer.MidTToCC),
        //new ExtendMedium(m_telescope, rumController, m_claw),
        new PivotEncoder(m_tower, Target3, m_claw, m_CubeShooter)
      ),
      new AutoAlign2(m_limelight, m_drivetrainSubsystem),
      new InstantCommand(m_claw::dropItem)
    );
  }
}
