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
  double ty, tv, tx, angle, distance, ta;
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

  public double getTy(){
    return ty;
  }

  public double getTa(){
    return ta;
  }

  /* 
  public double getDistance() {
    if (tv != 0) {
      angle = (Constants.ANGLE_CAMERA + ty) * Math.PI / 180;
      double a = ((Constants.HEIGHT_TARGET - Constants.HEIGHT_CAMERA) / Math.tan(angle)) / 12;
      return a;
    } else {
      return 0;
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


    // This method will be called once per scheduler run
  }
}
