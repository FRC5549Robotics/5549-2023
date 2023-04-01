// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4953 ;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.6858;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(1.41); 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 11; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(297.07); 

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 1; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 10; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(309.11); 

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(283.53); 
    public static final double armSpeed = 0.5;

    public static double INITIAL_HEADING;
    public static double POLE_DISTANCE = 39.75;

    //Auton
    
    //public PID Constants
    public static final double kP = 0.06;
    public static final double kI = 0.00;
    public static final double kD = 0.00;

    //Intake
    public static final int MOTOR_INTAKE_1 = 13;

    //Tower
    public static final int MOTOR_TOWER1 = 16;
    public static final int MOTOR_TOWER2 = 15;
    public static final double PIVOT_SPEED = 0.5;
    public static final double PIVOT_CONE_HIGH_SETPOINT = -0.580;
    public static final double PIVOT_CONE_MID_SETPOINT = -0.206;
    public static final double PIVOT_RETRACTED_SETPOINT = -0.02;
    public static final double PIVOT_INTAKE_SETPOINT = 0;
    public static final double PIVOT_CUBE_HIGH_SETPOINT = 0;
    public static final double PIVOT_CUBE_MID_SETPOINT = -0.171;

    //Telescope
    public static final int MOTOR_TELESCOPE_1 = 14;
    public static final double EXTEND_CONE_HIGH_SETPOINT = 7000;
    public static final double EXTEND_CONE_MID_SETPOINT = 4000;
    public static final double EXTEND_CUBE_HIGH_SETPOINT = 7000;
    public static final double EXTEND_CUBE_MID_SETPOINT = 4000;

    //Claw
    public static final int MOTOR_CLAW_1 = 17;
    public static final double CLAW_MOTOR_SPEED = 0.75;
    public static boolean yellow = true;

    //Cube Shooter
    public static final double CUBE_HINGE_HIGH_SETPOINT = 16.7;
    public static final double CUBE_HINGE_MID_SETPOINT = 0;
    public static final double CUBE_HINGE_LOW_AND_INTAKE_SETPOINT = 29.5;
    public static final double CUBE_HINGE_RETRACTED_SETPOINT = 15;
    public static final double CUBE_SHOOTER_HIGH_SETPOINT = 1;
    public static final double CUBE_SHOOTER_MID_SETPOINT = 0;
    public static final double CUBE_SHOOTER_LOW_SETPOINT = 0;
    public static final double CUBE_SHOOTER_INTAKE_SETPOINT = 0;
    public static final double CONE_PICKUP_FRONT = -0.213;
 
    //Pose Estimation
    public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(new Translation3d(Units.inchesToMeters(-6.5), Units.inchesToMeters(6.5), Units.inchesToMeters(20.5)), new Rotation3d(0, 0, 0));

}