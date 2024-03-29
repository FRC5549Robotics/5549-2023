// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

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
  private static final int TAG_TO_CHASE = 1;

  DrivetrainSubsystem drivetrainSubsystem;
  PhotonCamera photonCamera;
  PoseEstimator poseProvider;
  PoseEstimator poseEstimator;

  ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS);
  ProfiledPIDController yController = new ProfiledPIDController(2, 0, 0, Y_CONSTRAINTS);
  ProfiledPIDController omegaController = new ProfiledPIDController(1.5, 0, 0, OMEGA_CONSTRAINTS);
  
  private PhotonTrackedTarget lastTarget;
  /** Creates a new ChaseTag. */
  public ChaseTag(DrivetrainSubsystem DrivetrainSubsystem, PoseEstimator poseProvider) {
    drivetrainSubsystem = DrivetrainSubsystem;
    this.poseProvider = poseProvider;
    photonCamera = poseProvider.photonCamera;

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
    lastTarget = null;
    var robotPose = poseProvider.getCurrentPose();
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    omegaController.reset(robotPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    var robotPose2d = poseProvider.getCurrentPose();
    var robotPose = new Pose3d(robotPose2d.getX(), robotPose2d.getY(), 0, new Rotation3d(0,0,robotPose2d.getRotation().getRadians()));
    var photonRes = photonCamera.getLatestResult();
    if(photonRes.hasTargets()){
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
          .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2)
          .findFirst();
      
      if (targetOpt.isPresent()){
        var target = targetOpt.get();

        lastTarget = target;

        var cameraPose = robotPose.transformBy(Constants.ROBOT_TO_CAMERA);

        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);

        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());
      }
    } 
     
    if (lastTarget == null){
      drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
    } else {
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
