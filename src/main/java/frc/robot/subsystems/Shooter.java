// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IDs;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  Joysticks joysticks;
  TalonFX flyWheel = new TalonFX(IDs.flyWheelID);
  TalonFX nonFlyWheel = new TalonFX(IDs.rollerID);
  double RPM = 0.0;
  // boolean isXbox = false;
  
  public Shooter(Joysticks joysticks) {
    this.joysticks = joysticks;
  }

  public void setRPM(double RPM) {
    flyWheel.set(ControlMode.Velocity, RPM);
  }
  public void setRPMNoFLy(double RPM) {
    nonFlyWheel.set(ControlMode.Velocity, RPM);
  }
  @Override
  public void periodic() {   
    if (joysticks.getIncreaseShooter()) {
      if (RPM+Constants.increment<=7500) {
        setRPM(RPM+Constants.increment);
        setRPMNoFLy(RPM+Constants.increment);
      }
    }
    if (joysticks.getDecreaseShooter()) {
      if (RPM-Constants.increment>=0) {
        setRPM(RPM-Constants.increment);
        setRPMNoFLy(RPM-Constants.increment);
      }
    }
  }
}