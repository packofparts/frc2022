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
    public static final int climbTalonLeftID = 16;
    public static final int climbTalonRighID = 15;

    //SHOOTER TALON IDs
    public static final int flyWheelID = 6;
    public static final int rollerID = 7;

    public static final int intakeMotorID = 8;
    

    public static final int indexMotorID = 10;
    public static final int feederMotorID = 9;

    
    /*
    PNEUMATIC IDS
    */
    public static final int compressorID = 1;
    public static final int intakeSolenoid1ID = 0;
    public static final int intakeSolenoid2ID = 1;
    public static final int climbSolenoid1ID = 2;
    public static final int climbSolenoid2ID = 3;


    /*
    SENSOR IDs
    */
    public static final Port intakeColorSensorPort = I2C.Port.kOnboard;

    public static final int intakeUltrasonicFront = 1;
    public static final int intakeUltrasonicBack = 2;
}