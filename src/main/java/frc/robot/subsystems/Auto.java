package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Drive;

public class Auto extends SubsystemBase {
  Drive drive;
  PIDController pid = new PIDController(0, 0, 0);
  double desiredDistanceForShooting; //determined by what shooter decides
  double disFromBallErrorMargin = 1.0;
  XboxController xbox = new XboxController(Constants.xboxPort);
  Joystick joystick = new Joystick(Constants.joystickPort);
  Drive driveBase = new Drive();

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public Auto(Drive drive) {
    this.drive = drive;
    this.desiredDistanceForShooting = 0;
  }
 
  @Override
  public void periodic() {
    //1) Turn 180 degrees - Done
    double speed = pid.calculate(drive.yaw(), 180);

    driveBase.m_backLeftSpark.set(-speed);
    driveBase.m_frontLeftSpark.set(-speed);
    driveBase.m_backRightSpark.set(speed);
    driveBase.m_frontRightSpark.set(speed);


    //3) Go to predetermined distance




    //4) turn around again
  //if needs to turn around
    double speed2 = pid.calculate(drive.yaw(), -180);

    driveBase.m_backLeftSpark.set(-speed2);
    driveBase.m_frontLeftSpark.set(-speed2);
    driveBase.m_backRightSpark.set(speed2);
    driveBase.m_frontRightSpark.set(speed2);

    //driveToDeterminedPosition();  

    //4) Call shooter
    
    //https://docs.limelightvision.io/en/latest/cs_autorange.html
    //Code below is from case study: getting in range.

    //this code helps robot get in range before firing (at least that was what the video made it look like)
    //some of this stuff needs to be filled in
  }


/*  
public void driveToDeterminedPosition(){
    double left_command = 0;
    double right_command = 0;
    float KpDistance = -0.1f; //proportional control constant for distance
    if (joystick.getRawButton(9))
    {
      //unless stated otherwise, desiredDistanceForShooting = 0
      double distance_error = Math.abs(desiredDistanceForShooting - table.getEntry("ty").getDouble(right_command));
      double driving_adjust = KpDistance * distance_error;

      left_command += driving_adjust;
      right_command += driving_adjust;
    }

    float KpAim = -0.1f;

    float min_aim_command = 0.05f;

    double tx = table.getEntry("tx").getDouble(0);
    double ty = table.getEntry("ty").getDouble(0);

    if (joystick.getRawButton(9))
    {
      double heading_error = -tx;
      double distance_error = -ty;
      double steering_adjust = 0.0f;

      if (tx > 1.0){
        steering_adjust = KpAim*heading_error - min_aim_command;
      }
      else if (tx < 1.0){
        steering_adjust = KpAim*heading_error + min_aim_command;
      }

      double distance_adjust = KpDistance * distance_error;

      left_command += steering_adjust + distance_adjust;
      right_command -= steering_adjust + distance_adjust;
    }
  }
}
