// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class ChaseTag extends CommandBase {
  public static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(4, 3);
  public static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(4, 3);
  public static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

  static Transform3d TAG_TO_GOAL = new Transform3d(new Translation3d(1.5,0,0), new Rotation3d(0,0,Math.PI));

  DrivetrainSubsystem drivetrainSubsystem;
  NetworkTable apriltagNetworkTable;
  PoseEstimator poseSupplier;

  ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS);
  ProfiledPIDController yController = new ProfiledPIDController(2, 0, 0, Y_CONSTRAINTS);
  ProfiledPIDController omegaController = new ProfiledPIDController(1.5, 0, 0, OMEGA_CONSTRAINTS);
  
  /** Creates a new ChaseTag. */
  public ChaseTag(DrivetrainSubsystem DrivetrainSubsystem, PoseEstimator poseEstimator) {
    apriltagNetworkTable = poseEstimator.apriltagNetworkTable;
    drivetrainSubsystem = DrivetrainSubsystem;
    poseSupplier = poseEstimator;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI,Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DrivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d robotPose = poseSupplier.getCurrentPose();
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    omegaController.reset(robotPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose2d = poseSupplier.getCurrentPose();
    Pose3d robotPose = new Pose3d(robotPose2d.getX(), robotPose2d.getY(), 0, new Rotation3d(0,0,robotPose2d.getRotation().getRadians()));
    if(apriltagNetworkTable.getEntry("tv").getDouble(0) == 1){
      var cameraPose = robotPose.transformBy(Constants.CAMERA_TO_ROBOT.inverse());

      
      double[] transform = apriltagNetworkTable.getEntry("targetpose_cameraspace").getDoubleArray(new double [6]);
      var camToTarget = new Transform3d(new Translation3d(transform[0], transform[1], transform[2]), new Rotation3d(transform[3], transform[4], transform[5]));
      var targetPose = cameraPose.transformBy(camToTarget);

      var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

      xController.setGoal(goalPose.getX());
      yController.setGoal(goalPose.getY());
      omegaController.setGoal(goalPose.getRotation().getRadians());

      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()){
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()){
        ySpeed = 0;
      }

      var omegaSpeed = xController.calculate(robotPose2d.getRotation().getRadians());
      if (omegaController.atGoal()){
        omegaSpeed = 0;
      }

      drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed,omegaSpeed,robotPose2d.getRotation()));
    }
    else{
      drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
