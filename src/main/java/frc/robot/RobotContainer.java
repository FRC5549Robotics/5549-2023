// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.DefaultTelescopeCommand;
import frc.robot.commands.DefaultTowerCommand;
import frc.robot.commands.PivotEncoder;
import frc.robot.commands.AutoStable;
import frc.robot.commands.DefaultClawCommand;
import frc.robot.commands.DefaultCubeShooterCommand;
import frc.robot.commands.RunIntakeCube;
import frc.robot.commands.AutoAlignCommands.AutoAlign;
import frc.robot.commands.AutoAlignCommands.AutoAlign2;
import frc.robot.commands.AutoAlignCommands.AutoAlign2X;
import frc.robot.commands.AutoAlignCommands.AutoAlign2Y;
import frc.robot.commands.AutoAlignCommands.AutoAlign2Z;
import frc.robot.commands.AutonCommands.OneConeAuto;
import frc.robot.commands.AutonCommands.ThreeConeAuto;
import frc.robot.commands.AutonCommands.TwoConeAuto;
import frc.robot.commands.AutonCommands.ZeroConeAuto;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CubeShooter;
import frc.robot.Constants;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.subsystems.Limelight;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_controller2 = new XboxController(1);
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(m_controller);
  private final Limelight m_Limelight = new Limelight();
  private final Intake m_Intake = new Intake();
  private final Telescope m_telescope = new Telescope(m_controller);
  private final CubeShooter m_CubeShooter = new CubeShooter(m_controller2);
  private final Tower m_tower = new Tower();
  private AddressableLED led = new AddressableLED(0);
  private final Claw m_claw = new Claw(led);
  
  //All the Paths
  public static PathPlannerTrajectory BotToCC = PathPlanner.loadPath("BotToCC", new PathConstraints(4, 3));
  public static PathPlannerTrajectory BotToCT3 = PathPlanner.loadPath("BotToCT3", new PathConstraints(4, 3));
  public static PathPlannerTrajectory BotToCT4 = PathPlanner.loadPath("BotToCT4", new PathConstraints(4, 3));
  public static PathPlannerTrajectory CT1ToCC = PathPlanner.loadPath("CT1ToCC", new PathConstraints(4, 3));
  public static PathPlannerTrajectory CT1ToMidT = PathPlanner.loadPath("CT1ToMidT", new PathConstraints(4, 3));
  public static PathPlannerTrajectory CT1ToTop = PathPlanner.loadPath("CT1ToTop", new PathConstraints(4, 3));
  public static PathPlannerTrajectory CT2ToCC = PathPlanner.loadPath("CT2ToCC", new PathConstraints(4, 3));
  public static PathPlannerTrajectory CT2ToMidT = PathPlanner.loadPath("CT2ToMidT", new PathConstraints(4, 3));
  public static PathPlannerTrajectory CT2ToTop = PathPlanner.loadPath("CT2ToTop", new PathConstraints(4, 3));
  public static PathPlannerTrajectory CT3ToCC = PathPlanner.loadPath("CT3ToCC", new PathConstraints(4, 3));
  public static PathPlannerTrajectory CT3ToMidB = PathPlanner.loadPath("CT3ToMidB", new PathConstraints(4, 3));
  public static PathPlannerTrajectory CT3ToBot = PathPlanner.loadPath("CT3ToBot", new PathConstraints(4, 3));
  public static PathPlannerTrajectory CT4ToCC = PathPlanner.loadPath("CT4ToCC", new PathConstraints(4, 3));
  public static PathPlannerTrajectory CT4ToMidB = PathPlanner.loadPath("CT4ToMidB", new PathConstraints(4, 3));
  public static PathPlannerTrajectory CT4ToBot = PathPlanner.loadPath("CT4ToBot", new PathConstraints(4, 3));
  public static PathPlannerTrajectory MidBToCC = PathPlanner.loadPath("MidBToCC", new PathConstraints(4, 3));
  public static PathPlannerTrajectory MidBToCT3 = PathPlanner.loadPath("MidBToCT3", new PathConstraints(4, 3));
  public static PathPlannerTrajectory MidBToCT4 = PathPlanner.loadPath("MidBToCT4", new PathConstraints(4, 3));
  public static PathPlannerTrajectory MidTToCC = PathPlanner.loadPath("MidTToCC", new PathConstraints(4, 3));
  public static PathPlannerTrajectory MidTToCT1 = PathPlanner.loadPath("MidTToCT1", new PathConstraints(4, 3));
  public static PathPlannerTrajectory MidTToCT2 = PathPlanner.loadPath("MidTToCT2", new PathConstraints(4, 3));
  public static PathPlannerTrajectory TopToCC = PathPlanner.loadPath("TopToCC", new PathConstraints(4, 3));
  public static PathPlannerTrajectory TopToCT1 = PathPlanner.loadPath("TopToCT1", new PathConstraints(4, 3));
  public static PathPlannerTrajectory TopToCT2 = PathPlanner.loadPath("TopToCT2", new PathConstraints(4, 3));
  public static PathPlannerTrajectory SmallTest = PathPlanner.loadPath("SmallTest", new PathConstraints(0.5, 0.5));
  public static PathPlannerTrajectory MidCToCC = PathPlanner.loadPath("MidCToCC", new PathConstraints(4, 3));
  public static PathPlannerTrajectory BotCtoCC = PathPlanner.loadPath("BotCtoCC", new PathConstraints(4, 3));

  JoystickButton autoAlignButton = new JoystickButton(m_controller, 1);
  JoystickButton autoStableButton = new JoystickButton(m_controller, 2);


  JoystickButton towerCubeHighPosition = new JoystickButton(m_controller2, 2);
  JoystickButton towerCubeMidPosition = new JoystickButton(m_controller2, 1);
  JoystickButton intakePistonToggle = new JoystickButton(m_controller, 5);

  //AutoCommands
   Command m_ZeroConeAuto = new ZeroConeAuto(m_drivetrainSubsystem);
   Command m_OneConeAuto = new OneConeAuto(m_drivetrainSubsystem, m_telescope, m_tower, m_claw, m_controller, Tower.TargetLevel.CubeMid);
   Command m_TwoConeAuto = new  TwoConeAuto(m_drivetrainSubsystem, m_Intake, m_telescope, m_tower, m_Limelight, m_claw, m_controller, Tower.TargetLevel.CubeMid, Tower.TargetLevel.CubeMid);
   Command m_ThreeConeAuto = new ThreeConeAuto(m_drivetrainSubsystem, m_Intake, m_telescope, m_tower, m_Limelight, m_claw, m_controller, Tower.TargetLevel.CubeMid, Tower.TargetLevel.CubeMid, Tower.TargetLevel.CubeMid);

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));
    m_tower.setDefaultCommand(new DefaultTowerCommand(m_tower, m_controller2));
    m_telescope.setDefaultCommand(new DefaultTelescopeCommand(m_telescope, m_controller2));
    m_Intake.setDefaultCommand(new DefaultIntakeCommand(m_Intake, m_controller));
    m_claw.setDefaultCommand(new DefaultClawCommand(m_claw, m_Intake, m_controller2, led));
    m_CubeShooter.setDefaultCommand(new DefaultCubeShooterCommand(m_CubeShooter, m_controller));
    Constants.INITIAL_HEADING = m_drivetrainSubsystem.GetInitialHeading();
    SmartDashboard.putNumber("Initial Yaw", Constants.INITIAL_HEADING);
    // Configure the button bindings
    configureButtonBindings();

    //Adding Commands to autonomous command chooser
     m_autoChooser.setDefaultOption("Zero Cone Auto", m_ZeroConeAuto);
    m_autoChooser.addOption("One Cone Auto", m_OneConeAuto);
    m_autoChooser.addOption("Two Cone Auto", m_TwoConeAuto);
    m_autoChooser.addOption("Three Cone Auto", m_ThreeConeAuto);

    //Adding paths to path planner command chooser


    SmartDashboard.putData("Autonomous Command", m_autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Trigger(m_controller::getBackButton)
            // No requirements because we don't need to interrupt anything
            .onTrue(new RunCommand(m_drivetrainSubsystem::zeroGyroscope));
    autoAlignButton.whileTrue(new SequentialCommandGroup(
      //new AutoAlign2Z(m_Limelight, m_drivetrainSubsystem, m_controller)//,
      new AutoAlign2X(m_Limelight, m_drivetrainSubsystem)
      //new AutoAlign2Y(m_Limelight, m_drivetrainSubsystem, m_controller))
    ));
    autoStableButton.onTrue(new AutoStable(m_drivetrainSubsystem));

    //Intake Command

    //Claw Command


    //Tower-Position Command
    towerCubeMidPosition.whileTrue(new PivotEncoder(m_tower, Tower.TargetLevel.CubeMid));
    towerCubeHighPosition.whileTrue(new PivotEncoder(m_tower, Tower.TargetLevel.Retracted));

    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
  
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
