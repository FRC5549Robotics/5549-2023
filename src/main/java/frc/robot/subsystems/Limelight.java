// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */

  double Kp = 1/27;
  NetworkTable limelightTable;
  NetworkTable apriltagTable;
  double ty, tv, tx, angle, distance, yaw, ta;
  double min_command = 0.05;
  //XboxController xbox1;
  double steering_adjust = 0.0;
  private static Limelight limelight = null;
  

  public Limelight() {

    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public double getAngle() {
    SmartDashboard.putNumber("Horizontal Angle:", tx);
    if (tx != 0) {
      return tx;
    } else {
    return 0;
    }
  }

  public double getTx(){
    return tx;
  }

  public double getTa(){
    return ta;
  }

  public double getTy(){
    return ty;
  }

  public double getYaw(){
    return yaw;
  }

  public double calDistance()
  {
  

    double limelightMountAngleDegrees = 25.0;
    double limelightLensHeightInches = 20.0;
    double goalHeightInches = 60.0;

    double angleToGoalDegrees = limelightMountAngleDegrees + ty;
    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

    return distanceFromLimelightToGoalInches;
  }

  /* 
  public double getDistance() {
    if (tv != 0) {
      angle = (Constants.ANGLE_CAMERA + ty) * Math.PI / 180;
      double a = ((Constants.HEIGHT_TARGET - Constants.HEIGHT_CAMERA) / Math.tan(angle)) / 12;
      return a;
    } else {
      return 10;
    }
  }
*/

    public static Limelight getInstance(){
      return Limelight.limelight;
    }
    
  



  @Override
  public void periodic() {
    
   
 
    ty = limelightTable.getEntry("ty").getDouble(0);
    tv = limelightTable.getEntry("tv").getDouble(0);
    tx = limelightTable.getEntry("tx").getDouble(0);
    ta = limelightTable.getEntry("ta").getDouble(0);
    yaw = limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6])[4];

    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("tv", tv);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("yaw", yaw);
  

    // This method will be called once per scheduler run
  }
}

