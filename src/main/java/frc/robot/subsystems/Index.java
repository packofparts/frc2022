// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Index extends SubsystemBase {
  /** Creates a new Index. */

  TalonFX indexMotor = new TalonFX(IDs.indexMotorID);
  boolean isXbox = false;
  Joysticks joysticks;

  public Index(Joysticks joysticks) {
    this.joysticks = joysticks;
  }

  @Override
  public void periodic() {
    // if (operatorController.getPOV() == 0) {
    //   if (Constants.motorPower+Constants.increment<=7500) {
    //     indexMotor.set(ControlMode.Velocity, Constants.motorPower+Constants.increment);
    //   }
    // }
    // if (operatorController.getPOV() == 180) {
    //   if (Constants.motorPower-Constants.increment>=0) {
    //     indexMotor.set(ControlMode.Velocity, Constants.motorPower-Constants.increment);
    //   }
    // }
    
    if (joysticks.getIndexToggle()) setIndex(1);
    else if (joysticks.getOutdexToggle()) setIndex(-1);
    else setIndex(0);
  }

  public void setIndex(double speed) {
    indexMotor.set(ControlMode.PercentOutput, speed);
  }
}
