package frc.robot.constants;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class IDs {
    /*
    MOTOR IDS
    */
    //DRIVETRAIN SPARK IDs
    public static final int frontLeftSparkID = 1;
    public static final int frontRightSparkID = 2;
    public static final int backLeftSparkID = 3;
    public static final int backRightSparkID = 4;

    //CLIMB TALON IDs
    public static final int climbTalonID = 5;

    //SHOOTER TALON IDs
    public static final int flyWheelID = 6;
    public static final int rollerID = 7;

    public static final int intakeMotor1ID = 8;
    // public static final int intakeMotor2ID = 9;

    public static final int indexMotorID = 10;

    
    /*
    PNEUMATIC IDS
    */
    public static final int compressorID = 1;
    public static final int intakeSolenoid1ID = 0;
    public static final int intakeSolenoid2ID = 1;


    /*
    SENSOR IDs
    */
    public static final Port intakeColorSensorPort = I2C.Port.kOnboard;

    public static final int intakeUltrasonicPingPort = 1;
    public static final int intakeUltrasonicEchoPort = 2;

    public static final int indexUltrasonic1PingPort = 4;
    public static final int indexUltrasonic1EchoPort = 5;

    public static final int indexUltrasonic2PingPort = 6;
    public static final int indexUltrasonic2EchoPort = 7;
}