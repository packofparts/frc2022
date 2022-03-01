// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  Joysticks joysticks;
  TalonFX mainTalon = new TalonFX(IDs.flyWheelID);
  TalonFX rollerTalon = new TalonFX(IDs.rollerID);

  ShooterMode shooterMode = ShooterMode.off;
  
  public Shooter(Joysticks joysticks) {
    this.joysticks = joysticks;
  }

  public enum ShooterMode {
    fast, slow, off;
  }

  public void stopShooter() {
    this.shooterMode = ShooterMode.off;
    mainTalon.set(TalonFXControlMode.PercentOutput, 0);
    rollerTalon.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    //shooter toggle
    if (joysticks.getShooterHigh()) {
      if (shooterMode != ShooterMode.off) shooterMode = ShooterMode.off;
      else shooterMode = ShooterMode.fast;
    }
    else if (joysticks.getShooterLow()) {
      if (shooterMode != ShooterMode.off) shooterMode = ShooterMode.off;
      else shooterMode = ShooterMode.slow;
    }

    //shooter mode
    if (shooterMode == ShooterMode.fast) {
      mainTalon.set(TalonFXControlMode.PercentOutput, 0.5);
      rollerTalon.set(TalonFXControlMode.PercentOutput, -0.8);
    }
    else if (shooterMode == ShooterMode.slow) {
      mainTalon.set(TalonFXControlMode.PercentOutput, 0.42);
      rollerTalon.set(TalonFXControlMode.PercentOutput, -0.42);
    }
    else {
      mainTalon.set(TalonFXControlMode.PercentOutput, 0);
      rollerTalon.set(TalonFXControlMode.PercentOutput, 0);
    }
    
    SmartDashboard.putBoolean("Shooter Running", mainTalon.getMotorOutputPercent() != 0 || rollerTalon.getMotorOutputPercent() != 0);
    SmartDashboard.putNumber("Shooter-Main", mainTalon.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter-Roller", rollerTalon.getSelectedSensorVelocity());
  }
}