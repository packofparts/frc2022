// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;





public class Index extends SubsystemBase {
  /** Creates a new Index. */

  XboxController xbox = new XboxController(Constants.xboxPort);
  Joystick _joystick = new Joystick(Constants.joystickPort);
  public TalonFX indexMotor = new TalonFX(Constants.indexPort);
  boolean isXbox = false;

  public Index() {
    if(isXbox){
      if (xbox.getPOV() == 0) {
        if (Constants.motorPower+Constants.increment<=7500) {
          indexMotor.set(ControlMode.Velocity, Constants.motorPower+Constants.increment);
         
        }
      }
      if (xbox.getPOV() == 180) {
        if (Constants.motorPower-Constants.increment>=0) {
          indexMotor.set(ControlMode.Velocity, Constants.motorPower-Constants.increment);
          
        }
      }
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
