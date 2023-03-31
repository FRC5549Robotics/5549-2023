// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.io.IOException;
import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class PoseEstimator extends SubsystemBase {
  /** Creates a new PoseEstimator. */
  Limelight limelight;
  DrivetrainSubsystem drivetrainSubsystem;
  NetworkTable apriltagNetworkTable;

  static Vector<N7> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05, 0.05, 0.05);
  static Vector<N5> localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.01), 0.01, 0.01, 0.01, 0.01);
  static Vector<N3> visionMeaurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  SwerveDrivePoseEstimator poseEstimator;
  Field2d field2d = new Field2d();
  AprilTagFieldLayout layout;
  Pose3d[] poses;
  public PoseEstimator(Limelight Limelight, DrivetrainSubsystem DrivetrainSubsystem) {
    try {
      layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      // PV estimates will always be blue, they'll get flipped by robot thread
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
  
    } catch(IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
    for(int i = 0; i < 7; i++){
      Optional<Pose3d> pose = layout.getTagPose(i);
      if(pose.isPresent()){
        poses[i] = pose.get();
      }
    }
    limelight = Limelight;
    drivetrainSubsystem = DrivetrainSubsystem;
    apriltagNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
    poseEstimator = new SwerveDrivePoseEstimator(
      drivetrainSubsystem.m_kinematics, 
      drivetrainSubsystem.getGyroscopeRotation(), 
      new SwerveModulePosition[]{
        drivetrainSubsystem.m_frontLeftModule.getPosition(),
        drivetrainSubsystem.m_frontRightModule.getPosition(),
        drivetrainSubsystem.m_backLeftModule.getPosition(),
        drivetrainSubsystem.m_backRightModule.getPosition()}, 
      new Pose2d(), 
      visionMeaurementStdDevs, 
      visionMeaurementStdDevs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    apriltagNetworkTable.getEntry("getpipe").setDouble(2);
    Pose3d targetPose;
    Transform3d camToTarget;
    Pose3d camPose;
    var target = apriltagNetworkTable.getEntry("camerapose_targetspace").getInteger(0);
    if(target != 0){
      targetPose = new Pose3d(0, 0, 0, poses[(int)target + 1].getRotation());
      double[] transform = apriltagNetworkTable.getEntry("targetpose_cameraspace").getDoubleArray(new double [6]);
      camToTarget = new Transform3d(new Translation3d(transform[0], transform[1], transform[2]), new Rotation3d(transform[3], transform[4], transform[5]));
      camPose = targetPose.transformBy(camToTarget.inverse());
    }
  }
}
