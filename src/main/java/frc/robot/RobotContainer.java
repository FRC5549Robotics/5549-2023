// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultTelescopeCommand;
import frc.robot.commands.DefaultTowerCommand;
import frc.robot.commands.PipelineAutoAlign;
import frc.robot.commands.PivotEncoder;
import frc.robot.commands.AutoStable;
import frc.robot.commands.ChaseTag;
import frc.robot.commands.DefaultClawCommand;
import frc.robot.commands.DefaultCubeShooterCommand;
import frc.robot.commands.ShooterAim;
import frc.robot.commands.AutoAlignCommands.AutoAlign;
import frc.robot.commands.AutoAlignCommands.AutoAlign2;
import frc.robot.commands.AutoAlignCommands.AutoAlign2X;
import frc.robot.commands.AutoAlignCommands.AutoAlign2Y;
import frc.robot.commands.AutoAlignCommands.AutoAlign2Z;
import frc.robot.commands.AutonCommands.OneConeAutoNoDrive;
import frc.robot.commands.AutonCommands.OneConeChargeStation;
import frc.robot.commands.AutonCommands.OnlyChargeStation;
import frc.robot.commands.AutonCommands.OneConeAuto;
import frc.robot.commands.AutonCommands.ThreeConeAuto;
import frc.robot.commands.AutonCommands.TwoConeAuto;
import frc.robot.commands.AutonCommands.ZeroConeAuto;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CubeShooter;
import frc.robot.Constants;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimator;
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
  private final Telescope m_telescope = new Telescope(m_controller);
  private final Tower m_tower = new Tower();
  private final CubeShooter m_CubeShooter = new CubeShooter(m_tower);
  //private final PoseEstimator m_PoseEstimator = new PoseEstimator(m_Limelight, m_drivetrainSubsystem);
  private AddressableLED led = new AddressableLED(0);
  private final Claw m_claw = new Claw(led);
  private Limelight limelight = new Limelight();
  

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
  public static PathPlannerTrajectory CT4ToBotC = PathPlanner.loadPath("CT4ToBotC", new PathConstraints(4, 3));
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
  public static PathPlannerTrajectory MidBStraight = PathPlanner.loadPath("MidBStraight", new PathConstraints(4, 3));
  public static PathPlannerTrajectory TopAroundToCC = PathPlanner.loadPath("TopAroundToCC", new PathConstraints(4, 3));
  public static PathPlannerTrajectory BotAroundToCC = PathPlanner.loadPath("BotAroundToCC", new PathConstraints(4, 3));
  public static PathPlannerTrajectory MidTAroundToCC = PathPlanner.loadPath("MidTAroundToCC", new PathConstraints(4, 3));
  public static PathPlannerTrajectory MidBAroundToCC = PathPlanner.loadPath("MidBAroundToCC", new PathConstraints(1, 1));
  public static PathPlannerTrajectory TestCube = PathPlanner.loadPath("TestCube", new PathConstraints(2, 2));
  public static PathPlannerTrajectory CT1ToTopC = PathPlanner.loadPath("CT1ToTopC", new PathConstraints(4, 3));

  JoystickButton autoAlignButton = new JoystickButton(m_controller, 1);
  JoystickButton autoStableButton = new JoystickButton(m_controller, 2);
  JoystickButton chaseTag = new JoystickButton(m_controller, 3);
  JoystickButton resetNavXButton = new JoystickButton(m_controller, 4);

  JoystickButton towerConeHighPosition = new JoystickButton(m_controller2, 4);
  JoystickButton towerConeMidPosition = new JoystickButton(m_controller2, 2);
  JoystickButton stowedPosition = new JoystickButton(m_controller2, 1);
  JoystickButton towerConeFrontPickUpPosition = new JoystickButton(m_controller2, 3);

  //AutoCommands
   Command m_ZeroConeAutoMiddle = new ZeroConeAuto(m_drivetrainSubsystem, MidBStraight);
   Command m_ZeroConeAutoNearExit = new ZeroConeAuto(m_drivetrainSubsystem, BotToCT4);
   Command m_ZeroConeAutoNearWall = new ZeroConeAuto(m_drivetrainSubsystem, TopToCT1);
   Command m_OneConeAutoNoDrive = new OneConeAutoNoDrive(m_drivetrainSubsystem, m_telescope, m_tower, m_claw, m_CubeShooter, m_controller, Tower.TargetLevel.ConeHigh);
   Command m_OneConeAutoNearWall = new OneConeAuto(m_drivetrainSubsystem, m_telescope, m_tower, m_claw, m_CubeShooter, m_controller, Tower.TargetLevel.ConeHigh, TopToCT1);
   Command m_OneConeAutoNearExit = new OneConeAuto(m_drivetrainSubsystem, m_telescope, m_tower, m_claw, m_CubeShooter, m_controller, Tower.TargetLevel.ConeHigh, BotToCT4);
   Command m_TwoConeAuto = new  TwoConeAuto(m_drivetrainSubsystem, m_telescope, m_tower, m_Limelight, m_claw, m_CubeShooter, m_controller, TopToCT1, CT1ToTopC);
   Command m_ThreeConeAuto = new ThreeConeAuto(m_drivetrainSubsystem, m_telescope, m_tower, m_Limelight, m_claw, m_CubeShooter, m_controller, Tower.TargetLevel.ConeHigh, Tower.TargetLevel.CubeMid, Tower.TargetLevel.CubeMid);
   Command m_OneConeChargeStationNearWall = new OneConeChargeStation(m_drivetrainSubsystem, m_telescope, m_tower, m_claw, m_CubeShooter, m_controller, Tower.TargetLevel.ConeHigh, TopAroundToCC);
   Command m_OneConeChargeStationNearExit = new OneConeChargeStation(m_drivetrainSubsystem, m_telescope, m_tower, m_claw, m_CubeShooter, m_controller, Tower.TargetLevel.ConeHigh, BotAroundToCC);
   Command m_OneConeChargeStationMiddleWallSide = new OneConeChargeStation(m_drivetrainSubsystem, m_telescope, m_tower, m_claw, m_CubeShooter, m_controller, Tower.TargetLevel.ConeHigh, MidTAroundToCC);
   Command m_OneConeChargeStationMiddleExitSide = new OneConeChargeStation(m_drivetrainSubsystem, m_telescope, m_tower, m_claw, m_CubeShooter, m_controller, Tower.TargetLevel.ConeHigh, MidBAroundToCC);
   Command m_ChargeStationNearWall = new OnlyChargeStation(m_drivetrainSubsystem, TopAroundToCC);
   Command m_ChargeStationNearExit = new OnlyChargeStation(m_drivetrainSubsystem, BotAroundToCC);
   Command m_ChargeStationMiddleWallSide = new OnlyChargeStation(m_drivetrainSubsystem, MidTAroundToCC);
   Command m_ChargeStationMiddleExitSide = new OnlyChargeStation(m_drivetrainSubsystem, MidBAroundToCC);
   Command m_OneConeChargeStationNoCommunityMiddle = new OneConeChargeStation(m_drivetrainSubsystem, m_telescope, m_tower, m_claw, m_CubeShooter, m_controller, Tower.TargetLevel.ConeHigh, MidBToCC);
   Command m_TestCube = new ZeroConeAuto(m_drivetrainSubsystem, TestCube);
   Command m_TwoConeAutoNearSubstationWall = new TwoConeAuto(m_drivetrainSubsystem, m_telescope, m_tower, m_Limelight, m_claw, m_CubeShooter, m_controller, TopToCT1, CT1ToTopC);
   Command m_TwoConeAutoNearExitWall = new TwoConeAuto(m_drivetrainSubsystem, m_telescope, m_tower, m_Limelight, m_claw, m_CubeShooter, m_controller, BotToCT4, CT4ToBotC);

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /**pi
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
    m_claw.setDefaultCommand(new DefaultClawCommand(m_claw, m_controller2, led));
    m_CubeShooter.setDefaultCommand(new DefaultCubeShooterCommand(m_CubeShooter, m_controller, m_controller2));
    Constants.INITIAL_HEADING = m_drivetrainSubsystem.GetInitialHeading();
    SmartDashboard.putNumber("Initial Yaw", Constants.INITIAL_HEADING);
    // Configure the button bindings
    configureButtonBindings();

    //Adding Commands to autonomous command chooser
     m_autoChooser.setDefaultOption("Only Drive Middle", m_ZeroConeAutoMiddle);
    m_autoChooser.addOption("Only Drive Near Exit Wall", m_ZeroConeAutoNearExit);
    m_autoChooser.addOption("Only Drive Substation Wall", m_ZeroConeAutoNearWall);
    m_autoChooser.addOption("One Cone Auto Near Substation Wall", m_OneConeAutoNearWall);
    m_autoChooser.addOption("One Cone Auto Near Exit Wall", m_OneConeAutoNearExit);
    m_autoChooser.addOption("One Cone Auto No Drive", m_OneConeAutoNoDrive);
    m_autoChooser.addOption("One Cone Charge Station Substation Wall", m_OneConeChargeStationNearWall);
    m_autoChooser.addOption("One Cone Charge Station Exit Wall", m_OneConeChargeStationNearExit);
    m_autoChooser.addOption("One Cone Charge Station Middle Substation Side", m_OneConeChargeStationMiddleWallSide);
    m_autoChooser.addOption("One Cone Charge Station Middle Exit Side", m_OneConeChargeStationMiddleExitSide);
    m_autoChooser.addOption("Charge Station Substation Wall", m_ChargeStationNearWall);
    m_autoChooser.addOption("Charge Station Exit Wall", m_ChargeStationNearExit);
    m_autoChooser.addOption("Charge Station Middle Substation Side", m_ChargeStationMiddleWallSide);
    m_autoChooser.addOption("Charge Station Middle Exit Side", m_ChargeStationMiddleExitSide);
    m_autoChooser.addOption("One Cone Charge Station No Community Middle Exit Side", m_OneConeChargeStationNoCommunityMiddle);
    m_autoChooser.addOption("Two Cone Auto Near Substation Wall", m_TwoConeAutoNearSubstationWall);
    m_autoChooser.addOption("Two Cone Auto Near Exit Wall", m_TwoConeAutoNearExitWall);
    m_autoChooser.addOption("Do Nothing", null);

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
    // No requirements because we don't need to interrupt anything
    resetNavXButton.onTrue(new InstantCommand(m_drivetrainSubsystem::zeroGyroscope));
    //autoAlignButton.whileTrue(new SequentialCommandGroup(
      //new AutoAlign2Z(m_Limelight, m_drivetrainSubsystem, m_controller)//,
    //  new AutoAlign2X(m_Limelight, m_drivetrainSubsystem)
      //new AutoAlign2Y(m_Limelight, m_drivetrainSubsystem, m_controller))
    //));
    //chaseTag.whileTrue(new ChaseTag(m_drivetrainSubsystem, m_PoseEstimator));
    autoStableButton.whileTrue(new AutoStable(m_drivetrainSubsystem));

    autoAlignButton.whileTrue(new PipelineAutoAlign(limelight, m_drivetrainSubsystem));

    //Intake Command

    //Claw Command
    //toggleClaw.onTrue(new toggleClaw(m_claw));

    //Tower-Position Command
    towerConeMidPosition.whileTrue(new ParallelCommandGroup(
      new PivotEncoder(m_tower, Tower.TargetLevel.ConeMid, m_claw, m_CubeShooter),
      new ));
    towerConeHighPosition.whileTrue(new PivotEncoder(m_tower, Tower.TargetLevel.ConeHigh, m_claw, m_CubeShooter));
    towerConeFrontPickUpPosition.whileTrue(new PivotEncoder(m_tower, Tower.TargetLevel.PickUpFront, m_claw, m_CubeShooter));
    stowedPosition.whileTrue(new PivotEncoder(m_tower, Tower.TargetLevel.Retracted, m_claw, m_CubeShooter));
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
