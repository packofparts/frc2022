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
    public static final double joystickDeadzone = 0.1;

    public static final double pGyro = 0.01;
    public static final Gains defaultPID = new Gains(0.05, 0.00001, 0.7, 0.0, 0.0, -0.5, 0.5, 0);
    
    public static final double maxSpeed = 6;
    public static final double maxTurnOutput = Math.toRadians(5);
    public static final double minTurnInput = 0.1;
    public static final double wheelDisFromCenter = 0.5;

    public static final int frontLeftSparkID = 1;
    public static final int frontRightSparkID = 2;
    public static final int backLeftSparkID = 3;
    public static final int backRightSparkID = 4;
}