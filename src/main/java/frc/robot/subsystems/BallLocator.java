package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class BallLocator extends SubsystemBase {}
  Drive drive;
  PIDController pid = new PIDController(0, 0, 0);
  double desiredDistanceForShooting; //determined by what shooter decides
  double disFromBallErrorMargin = 1.0;
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public BallLocator(Drive drive, double desiredDistanceForShooting) {
    this.drive = drive;
    this.desiredDistanceForShooting = desiredDistanceForShooting;
  }
 
  @Override
  public void periodic() {
    //1) Turn 180 degrees [good]
    pid.calculate(drive.yaw(), 180);

    //2) Go till within disFromBallErrorMargin distance

    //3) Go to predetermined distance
    pid.calculate(drive.yaw(), -180);  //if needs to turn around
    driveToDeterminedPosition();  

    //4) Call shooter
    
    //https://docs.limelightvision.io/en/latest/cs_autorange.html
    //Code below is from case study: getting in range.

    //this code helps robot get in range before firing (at least that was what the video made it look like)
    //some of this stuff needs to be filled in
  }
  
public void driveToDeterminedPosition(){
    double left_command = 0;
    double right_command = 0;

    float KpDistance = -0.1f; //proportional control constant for distance
    if (joystick.GetRawButton(9))
    {
      float distance_error = Math.abs(desiredDistanceForShooting - table.GetEntry("ty").getDouble());
      driving_adjust = KpDistance * distance_error;

      left_command += distance_adjust;
      right_command += distance_adjust;
    }

    float KpAim = -0.1f;
    float KpDistance = -0.1f;
    float min_aim_command = 0.05f;

    float tx = table.GetEntry("tx").getDouble(0);
    float ty = table.GetEntry("ty").getDouble(0);

    if (joystick.GetRawButton(9))
    {
      float heading_error = -tx;
      float distance_error = -ty;
      float steering_adjust = 0.0f;

      if (tx > 1.0){
        steering_adjust = KpAim*heading_error - min_aim_command;
      }
      else if (tx < 1.0){
        steering_adjust = KpAim*heading_error + min_aim_command;
      }

      float distance_adjust = KpDistance * distance_error;

      left_command += steering_adjust + distance_adjust;
      right_command -= steering_adjust + distance_adjust;
    }
  }
}
