// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.revrobotics.ColorSensorV3.RawColor;

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
    public static final double joystickDeadzone = 0.25;
    //maximum speed of the robot in m/s
    public static final double maxSpeed = 5000;
    //maximum rotation of the robot in rads/sec
    public static final double maxRate = 4;
    //gyrohold rate correction aggressiveness
    public static final double rateAggresiveness = 1;
    public static final double rateFactor = -4.0;
    //maximum rotation of the robot in rads/sec
    public static final double maxTurnOutput = Math.toRadians(180)*1000;
    //distance of wheels from center of robot
    public static final double wheelDisFromCenter = 0.5715;
    //distance of cg from robot center
    public static final Translation2d cgDistance = new Translation2d(0, 0.5);
    //proportional adjustment for gyro pid
    public static final double pGyro = 0.01;
    //gyro pid turn margin of error
    public static final double gyroDeadzone = 1;

    //precision factor for precision turning
    public static final double precisionFactor = 0.4;
    //input exponent factor
    public static final double exponentFactor = 4/3;
    //input exponent factor for turning
    public static final double turnExponentFactor = 5/3;
    //min forward/side power input for movement
    public static final double minPower = 0.1;
    //min rotation input for turning
    public static final double minTurn = 0.2;
    /*
    Tube Sensor Constants
    */
    public static final double minColorSensorDis = 70;
    public static final RawColor redBallThreshold = new RawColor(0, 0, 0, 0);
    public static final RawColor blueBallThreshold = new RawColor(0, 0, 0, 0);
    public static final double indexUltrasonicFrontThreshold = 0;
    public static final double indexUltrasonicBackThreshold = 0;

    /*
    Shooter Constants
    */
    public static final double shooterDeadzone = 50; 
    //shooter main RPM PID
    public static final double[] shooterMainPID = new double[] {0.8, 0.0, 0.0, 0.0521852887938127365476386374856};//8
    //shooter roller RPM PID
    public static final double[] shooterRollerPID = new double[] {0.125, 0.0, 0.0, 0.048};
    //125


    /*
    Vision Constants
    */
    public static final double tvert = 80.0;
    public static final double thor = 140.0;
    public static final double tarea = 16.5;


    /*
    Encoder Conversions
    */
    //drive conversion
    public static final double encoderConversion = 0.12345;
    //drive position deadzone
    public static final double encoderDeadzone = 0.05;
    //climb encoder conversion
    public static final double climbFactor = 1;


    /*
    PID Settings
    */
    //drive wheel velocity PID
    public static final Gains velocityPID = new Gains(0.0002, 0.0000005, 0.0, 0.0, 0.0, -1, 1, 0);
    //drive position movement PID
    public static final Gains movePID = new Gains(0.10, 0.0, 0.0, 0.0, 0.0, -1, 1, 1);
    //drive limelight move PID
    public static final double[] limelightPID = new double[] {0.00001, 0.0, 0.0};
    public static final double[] limelightMovePID = new double[] {0.125, 0.0, 0.01};
    public static final double[] limelightTurnPID = new double[] {0.017, 0.0, 0.002};
    //drive gyro turn PID
    public static final double[] turnPID = new double[] {0.006, 0.0, 0.0003};
    public static final double turnPIDTolerance = 0.5;
    //drive gyro rate PID
    public static final double[] ratePID = new double[] {0.080, 0.0, 0.0};

}