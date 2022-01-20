// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;


public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  XboxController joystick = new XboxController(0);
  Joystick _joystick = new Joystick(0);
  TalonFX shootMotor = new TalonFX(0);
  //PIDSetConfiguration
  double RPM=0.0;
  boolean isXbox = true;
  
  public Shooter() {
    if(isXbox){
      if (joystick.getPOV() == 0) {
        if (RPM+Constants.increment<=7500) {
          setRPM(RPM+Constants.increment);
        }
      }
      if (joystick.getPOV() == 180) {
        if (RPM-Constants.increment>=0) {
          setRPM(RPM-Constants.increment);
        }
      }
    }

    else {

      double stick = _joystick.getRawAxis(2);
      shootMotor.set(ControlMode.Velocity, -stick);
    }
  }

  public void setRPM(double RPM) {
    shootMotor.set(ControlMode.Velocity, RPM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}