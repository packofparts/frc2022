
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //deadzone for the joysticks before input is used

    

    //IDs of CAN objects
    public static final double kpLime = 0.1; //TBD
    public static final double kiLime = 0; //TBD
    public static final double kdLime = 0.5; // TBD
    public static final int autoAlignButton = 4;
    public static final int xboxPort = 0;
    public static final int joystickPort = 0;
    public static final int flyWheelPort = 0;
    public static final int indexPort = 0;
    public static double increment=0.1;
    public static double PIDTurnDegrees=10.0;
    public static double indexSpeed;
    public static final double motorPower = 0.2;
    public static final double spinTime = 2.0;//tbd
    public static final double tvert = 105.0; //experimental
    public static final double thor = 156.0; //experimental

    public static final double LimelightKP = 0;

    public static final double LimelightKI = 0;

    public static final double LimelightKD = 0;

    public static final double joystickDeadzone = 0.2;

    //Proportional adjustment for gyro pid
    public static final double pGyro = 0.01;
    //Gyro rotation rate deadzone before snap is enabled
    public static final double gyroDeadzone = 3;
    //encoder conversion
    public static final double encoderConversion = 0.14771805182393566186414699677227;
    //encoder position deadzone
    public static final double encoderDeadzone = 0.5;

    //PID settings
    public static final Gains velocityPID = new Gains(0.0002, 0.0000005, 0.0, 0.0, 0.0, -1, 1, 0);
    public static Gains defaultPID = new Gains(0.2, 0.0, 0.0, 0.0, 0.0, -1, 1, 1);
    public static final double[] turnPID = new double[] {0.009, 0.0, 0.002};
    public static final double[] ratePID = new double[] {0.15, 0.008, 0.0};
    //maximum speed of the robot in m/s
    public static final double maxSpeed = 5000;
    //maximum rotation of the robot in rads/sec
    public static final double maxTurnOutput = Math.toRadians(180)*1000;
    //maximum rate of the robot in degrees/sec
    public static final double maxRate = 1;
    //minimum input required for robot to move
    public static final double minInput = 0.1;
    //distance of wheels from center of robot
    public static final double wheelDisFromCenter = 0.5715;
    //distance of cg from robot center
    public static final Translation2d cgDistance = new Translation2d(0, 0.5);

    //IDs of CAN objects
    public static final int frontLeftSparkID = 1;
    public static final int frontRightSparkID = 2;
    public static final int backLeftSparkID = 3;
    public static final int backRightSparkID = 4;
}