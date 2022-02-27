// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IDs;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  Joysticks joysticks;
  TalonFX mainTalon = new TalonFX(IDs.flyWheelID);
  TalonFX rollerTalon = new TalonFX(IDs.rollerID);
  double RPM = 0.0;

  boolean shooterHigh = false;
  boolean shooterLow = false;
  // boolean isXbox = false;
  
  public Shooter(Joysticks joysticks) {
    this.joysticks = joysticks;
  }

  public void setRPM(double RPM) {
    mainTalon.set(ControlMode.Velocity, RPM);
  }
  public void setRPMRoller(double RPM) {
    rollerTalon.set(ControlMode.Velocity, RPM);
  }

  public void stopShooter() {
    shooterLow = false;
    shooterHigh = false;
  }

  @Override
  public void periodic() {   
    // SmartDashboard.putNumber("Shooter-main", mainTalon.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Shooter-roller", mainTalon.getSelectedSensorVelocity());

    // double mainShooter = joysticks.getShooterMain();
    // double rollerShooter = joysticks.getShooterRoller();
    // if (mainShooter < Constants.joystickDeadzone) mainShooter = 0;
    // if (rollerShooter < Constants.joystickDeadzone) rollerShooter = 0;
    // mainTalon.set(TalonFXControlMode.PercentOutput, mainShooter);
    // rollerTalon.set(TalonFXControlMode.PercentOutput, rollerShooter);

    if (joysticks.getShooterHigh()) {
      shooterHigh = !shooterHigh;
      shooterLow = false;
    }
    else if (joysticks.getShooterLow()) {
      shooterHigh = false;
      shooterLow = !shooterLow;
    }

    if (shooterHigh) {
      mainTalon.set(TalonFXControlMode.PercentOutput, 0.5);
      rollerTalon.set(TalonFXControlMode.PercentOutput, -0.8);
    }
    else if (shooterLow) {
      mainTalon.set(TalonFXControlMode.PercentOutput, 0.42);
      rollerTalon.set(TalonFXControlMode.PercentOutput, -0.42);
    }
    else {
      mainTalon.set(TalonFXControlMode.PercentOutput, 0);
      rollerTalon.set(TalonFXControlMode.PercentOutput, 0);
    }
    // if (joysticks.getIncreaseShooter()) {
    //   if (RPM+Constants.increment<=7500) {
    //     setRPM(RPM+Constants.increment);
    //     setRPMRoller(RPM+Constants.increment);
    //   }
    // }
    // if (joysticks.getDecreaseShooter()) {
    //   if (RPM-Constants.increment>=0) {
    //     setRPM(RPM-Constants.increment);
    //     setRPMRoller(RPM-Constants.increment);
    //   }
    // }
  }
}