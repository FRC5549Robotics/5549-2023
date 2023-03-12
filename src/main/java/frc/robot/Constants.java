// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 10; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(5.63); 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 11; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(297.33); 

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 1; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 9; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(305.60); 

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(284.24); 
    public static final double armSpeed = 0.1;

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

    //Telescope
    public static final int MOTOR_TELESCOPE_1 = 14;

    //Claw
    public static final int MOTOR_CLAW_1 = 17;
    public static final double CLAW_MOTOR_SPEED = 0.75;
}