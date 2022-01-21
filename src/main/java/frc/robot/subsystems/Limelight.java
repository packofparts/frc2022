// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//test commment
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.*;
public class Lidar extends SubsystemBase {

  /** Creates a new Lidar. */ 
  public Lidar() {
    float Kp = -0.1f; //proportional increment to adjust heading
    float min_command = 0.05f; // minimum increment to get response from drivebase


  //Accesses the tx value from network table ( Horizontal offset)
    std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
    float tx = table.GetNumber("tx");

    if (joystick->GetRawButton(9))
    {
      float heading_error = -tx;
      float steering_adjust = 0.0f;
      if (tx > 1.0)
      {
              steering_adjust = Kp*heading_error - min_command;
      }
      else if (tx < 1.0)
      {
        steering_adjust = Kp*heading_error + min_command;
      }
      left_command += steering_adjust;
      right_command -= steering_adjust;
    }




  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
