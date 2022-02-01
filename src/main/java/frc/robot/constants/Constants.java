// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /*
    Drive Constants
    */
    //deadzone for the joysticks before input is used
    public static final double joystickDeadzone = 0.2;
    //maximum speed of the robot in m/s
    public static final double maxSpeed = 5000;
    //maximum rotation of the robot in rads/sec
    public static final double maxRate = 3;
    //maximum rotation of the robot in rads/sec
    public static final double maxTurnOutput = Math.toRadians(180)*1000;
    //minimum input required for robot to move
    public static final double minInput = 0.1;
    //distance of wheels from center of robot
    public static final double wheelDisFromCenter = 0.5715;
    //distance of cg from robot center
    public static final Translation2d cgDistance = new Translation2d(0, 0.5);
    //Proportional adjustment for gyro pid
    public static final double pGyro = 0.01;
    //Gyro rotation rate deadzone before snap is enabled
    public static final double gyroDeadzone = 3;


    /*
    Shooter Constants
    */
    public static final double increment = 100;


    /*
    Vision Constants
    */
    public static final double tvert = 105.0;
    public static final double thor = 156.0;
    public static final double tarea = 16.5;


    /*
    Encoder Conversions
    */
    //drive conversion
    public static final double encoderConversion = 0.14771805182393566186414699677227;
    //drive position deadzone
    public static final double encoderDeadzone = 0.5;
    //climb encoder conversion
    public static final double climbFactor = 1;


    /*
    PID Settings
    */
    //drive wheel velocity PID
    public static final Gains velocityPID = new Gains(0.0002, 0.0000005, 0.0, 0.0, 0.0, -1, 1, 0);
    //drive position movement PID
    public static final Gains movePID = new Gains(0.2, 0.0, 0.0, 0.0, 0.0, -1, 1, 1);
    //drive gyro turn PID
    public static final double[] turnPID = new double[] {0.009, 0.0, 0.002};
    //drive gyro rate PID
    public static final double[] ratePID = new double[] {0.125, 0.0, 0.0};

    
    /*
    MISC Constants
    */
    public static final double spinTimeIntake = 0;
    public static final double spinTimeOuttake = 0;
    public static final int ultrasonicThreshold = 5;
    
    public static final double motorPower = 0.2;
    public static final double spinTime = 2.0;//tbd
}