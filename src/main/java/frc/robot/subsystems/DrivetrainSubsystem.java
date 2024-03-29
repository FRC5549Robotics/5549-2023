// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.revrobotics.RelativeEncoder;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import frc.lib.MkSwerveModuleBuilder;
import frc.lib.SdsModuleConfigurations;
import frc.lib.SwerveModule;
import frc.robot.Constants;
import frc.lib.MotorType;
import edu.wpi.first.math.geometry.Pose2d;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * T/''/he maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L1.getDriveReduction() *
          SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );
  public final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); 

  // These are our modules. We initialize them in the constructor.
  final SwerveModule m_frontLeftModule;
  final SwerveModule m_frontRightModule;
  final SwerveModule m_backLeftModule;
  final SwerveModule m_backRightModule;

  public RelativeEncoder m_frontLeftModuleDriveEncoder, m_frontLeftModuleSteerEncoder;
  public RelativeEncoder m_frontRightModuleDriveEncoder, m_frontRightModuleSteerEncoder;
  public RelativeEncoder m_backLeftModuleDriveEncoder, m_backLeftModuleSteerEncoder;
  public RelativeEncoder m_backRightModuleDriveEncoder, m_backRightModuleSteerEncoder;

  public boolean locked = false;

  public ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  SwerveDriveOdometry m_odometry;
  XboxController m_Controller;
  public DrivetrainSubsystem(XboxController xbox) {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    m_Controller = xbox;
     
    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    m_frontLeftModule = new MkSwerveModuleBuilder()
    .withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0))
    .withGearRatio(SdsModuleConfigurations.MK4_L1)
    .withDriveMotor(MotorType.NEO, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR)
    .withSteerMotor(MotorType.NEO, Constants.FRONT_LEFT_MODULE_STEER_MOTOR)
    .withSteerEncoderPort(Constants.FRONT_LEFT_MODULE_STEER_ENCODER)
    .withSteerOffset(Constants.FRONT_LEFT_MODULE_STEER_OFFSET)
    .build();

    // We will do the same for the other modules
    m_frontRightModule = new MkSwerveModuleBuilder()
    .withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0))
    .withGearRatio(SdsModuleConfigurations.MK4_L1)
    .withDriveMotor(MotorType.NEO, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR)
    .withSteerMotor(MotorType.NEO, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR)
    .withSteerEncoderPort(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER)
    .withSteerOffset(Constants.FRONT_RIGHT_MODULE_STEER_OFFSET)
    .build();

    m_backLeftModule = new MkSwerveModuleBuilder()
    .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0))
    .withGearRatio(SdsModuleConfigurations.MK4_L1)
    .withDriveMotor(MotorType.NEO, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR)
    .withSteerMotor(MotorType.NEO, Constants.BACK_LEFT_MODULE_STEER_MOTOR)
    .withSteerEncoderPort(Constants.BACK_LEFT_MODULE_STEER_ENCODER)
    .withSteerOffset(Constants.BACK_LEFT_MODULE_STEER_OFFSET)
    .build();

    m_backRightModule = new MkSwerveModuleBuilder()
    .withLayout(tab.getLayout(" Back Right", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0))
    .withGearRatio(SdsModuleConfigurations.MK4_L1)
    .withDriveMotor(MotorType.NEO, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR)
    .withSteerMotor(MotorType.NEO, Constants.BACK_RIGHT_MODULE_STEER_MOTOR)
    .withSteerEncoderPort(Constants.BACK_RIGHT_MODULE_STEER_ENCODER)
    .withSteerOffset(Constants.BACK_RIGHT_MODULE_STEER_OFFSET)
    .build();

    m_odometry = new SwerveDriveOdometry(
        m_kinematics,
        getGyroscopeRotation(),
        new SwerveModulePosition[]{
                m_frontLeftModule.getPosition(),
                m_frontRightModule.getPosition(),
                m_backLeftModule.getPosition(),
                m_backRightModule.getPosition()
        }
    );
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
        //m_navx.reset();
        /* 
    m_odometry.resetPosition(
        getGyroscopeRotation(), 
        new SwerveModulePosition[]{m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(), m_backLeftModule.getPosition(), m_backRightModule.getPosition()}, 
        new Pose2d(m_odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0))
    );
    */
        m_navx.reset(); 
  }

  public Rotation2d getGyroscopeRotation() {
    // FIXME Uncomment if you are using a NavX
    if (m_navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }
//
//    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public Pose2d getPose(){
        return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
        m_odometry.resetPosition(
                getGyroscopeRotation(), 
                new SwerveModulePosition[]{
                        m_frontLeftModule.getPosition(),
                        m_frontRightModule.getPosition(),
                        m_backLeftModule.getPosition(),
                        m_backRightModule.getPosition()
                }, 
                pose
                );
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }
  
  // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
public Command followTrajectoryCommand(PathPlannerTrajectory traj) {
             return new PPSwerveControllerCommand(
                 traj, 
                 this::getPose, // Pose supplier
                 this.m_kinematics, // SwerveDriveKinematics
                 new PIDController(1, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 new PIDController(3.5, 0, 0), // Y controller (usually the same values as X controller)
                 new PIDController(1.7, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 (SwerveModuleState[] states) -> {
                        this.m_chassisSpeeds = this.m_kinematics.toChassisSpeeds(states);
                }, // Module states consumer
                 true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                 this // Requires this drive subsystem
             );
     }
public double GetInitialHeading(){
        return m_navx.getYaw();
}

public double getCurrentHeading(){
        return m_navx.getAngle();
}

public double getCurrentPitch(){
        return m_navx.getPitch();
}

public void lockModules() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = new SwerveModuleState(0,Rotation2d.fromDegrees(315));
    states[1] = new SwerveModuleState(0,Rotation2d.fromDegrees(45));
    states[2] = new SwerveModuleState(0,Rotation2d.fromDegrees(225));
    states[3] = new SwerveModuleState(0,Rotation2d.fromDegrees(135));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    //Speed will just be 0
    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());    
}

  public boolean getGyroConnected(){
        return m_navx.isConnected();
  }

  @Override
  public void periodic() {
        SmartDashboard.putBoolean("Navx Connected?", getGyroConnected());
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

                SmartDashboard.putNumber("Navx Pitch", m_navx.getPitch());
                SmartDashboard.putNumber("Navx Yaw", m_navx.getYaw());
                m_odometry.update(
                getGyroscopeRotation(),
                new SwerveModulePosition[]{
                        m_frontLeftModule.getPosition(),
                        m_frontRightModule.getPosition(),
                        m_backLeftModule.getPosition(),
                        m_backRightModule.getPosition()
                });
                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
  }
}
