// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double increment = 750;
    //deadzone for the joysticks before input is used
    public static final double joystickDeadzone = 0.2;

    public static final int kShooterPort = 0;
    public static final double increment = 750;
    public static final double joystickDeadzone = 0.1;

    //Proportional adjustment for gyro pid
    public static final double pGyro = 0.01;
    //Gyro rotation rate deadzone before snap is enabled
    public static final double gyroDeadzone = 3;
    //encoder conversion
    public static final double encoderConversion = 0.14771805182393566186414699677227;
    //encoder position deadzone
    public static final double encoderDeadzone = 0.5;
    //encoder conversion for Climb motor
    public static final double climbFactor = 1;

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

    //PID settings
    public static final Gains velocityPID = new Gains(0.0002, 0.0000005, 0.0, 0.0, 0.0, -1, 1, 0);
    public static Gains defaultPID = new Gains(0.2, 0.0, 0.0, 0.0, 0.0, -1, 1, 1);
    public static final double[] turnPID = new double[] {0.009, 0.0, 0.002};
    public static final double[] ratePID = new double[] {0.125, 0.0, 0.0};
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

    //IDs of CAN objects
    public static final int frontLeftSparkID = 1;
    public static final int frontRightSparkID = 2;
    public static final int backLeftSparkID = 3;
    public static final int backRightSparkID = 4;

    public static final int climbTalonID = 5;

    public static Joystick joystick = new Joystick(0);

    public static final int kShooterPort = 0;
    public static final double increment = 0.2;
    public static final int intakeMotor1Port = 0;
    public static final int intakeMotor2Port = 1;
    public static final Port intakeColorSensorPort = I2C.Port.kOnboard;
    public static final int intakeUltrasonicPingPort = 1;
    public static final int intakeUltrasonicEchoPort = 2;

    public static final int indexMotor = 0;

    public static final int ultrasonicThreshold = 5;

    public static final int indexUltrasonic1PingPort = 4;

    public static final int indexUltrasonic2PingPort = 5;

    public static final int indexUltrasonic1EchoPort = 6;

    public static final int indexUltrasonic2EchoPort = 7;

    public static final int shooterTalonID = 6;
}