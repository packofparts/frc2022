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

    public static final int kShooterPort = 0;
    public static final double increment = 750;
    public static final double joystickDeadzone = 0.1;

    //Proportional adjustment for gyro pid
    public static final double pGyro = 0.01;
    //Gyro rotation rate deadzone before snap is enabled
    public static final double gyroDeadzone = 3;

    //PID settings
    public static final Gains defaultPID = new Gains(0.05, 0.00001, 0.7, 0.0, 0.0, -0.5, 0.5, 0);
    
    //maximum speed of the robot in m/s
    public static final double maxSpeed = 6;
    //maximum rotation of the robot in rads/sec
    public static final double maxTurnOutput = Math.toRadians(5);
    public static final double minTurnInput = 0.1;
    //minimum input required for robot to move
    public static final double minInput = 0.1;
    //distance of wheels from center of robot
    public static final double wheelDisFromCenter = 0.5715;

    //IDs of CAN objects
    public static final int frontLeftSparkID = 1;
    public static final int frontRightSparkID = 2;
    public static final int backLeftSparkID = 3;
    public static final int backRightSparkID = 4;
    public static final double kpLime = -0.1;
    public static final double kiLime = 0;
    public static final double kdLime = 0;
    public static final int autoAlignButton = 0;
    public static final int joystickPort = 0;




}
