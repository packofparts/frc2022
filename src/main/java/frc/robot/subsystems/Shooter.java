// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IDs;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  Joysticks joysticks;
  TalonFX mainTalon = new TalonFX(IDs.flyWheelID);
  TalonFX rollerTalon = new TalonFX(IDs.rollerID);

  ShooterMode shooterMode = ShooterMode.off;
  boolean shooterReady = false;
  double setVelocityMain = 0;
  double setVelocityRoller = 0;

  boolean usePID = false;
  
  public Shooter(Joysticks joysticks) {
    this.joysticks = joysticks;

    mainTalon.config_kP(0, Constants.shooterMainPID[0]);
    mainTalon.config_kI(0, Constants.shooterMainPID[1]);
    mainTalon.config_kD(0, Constants.shooterMainPID[2]);
    
    rollerTalon.config_kP(0, Constants.shooterRollerPID[0]);
    rollerTalon.config_kI(0, Constants.shooterRollerPID[1]);
    rollerTalon.config_kD(0, Constants.shooterRollerPID[2]);
  }

  public enum ShooterMode {
    fast, slow, auto, off;
  }

  @Override
  public void periodic() {
    //shooter toggle
    if (joysticks.getShooterHigh()) {
      if (shooterMode != ShooterMode.off) setShooterMode(ShooterMode.off);
      else setShooterMode(ShooterMode.fast);
    }
    else if (joysticks.getShooterLow()) {
      if (shooterMode != ShooterMode.off) setShooterMode(ShooterMode.off);
      else setShooterMode(ShooterMode.slow);
    }

    //shooter mode
    runShooter();
  }

  public boolean getShooterReady() {
    return shooterReady;
  }
  
  public void setShooterMode(ShooterMode mode) {
    this.shooterMode = mode;
  }

  public void stopShooter() {
    this.shooterMode = ShooterMode.off;
    runShooter();
  }

  public void runShooter() {
    if (!usePID) {
      if (shooterMode == ShooterMode.fast) {
        mainTalon.set(TalonFXControlMode.PercentOutput, 0.5);
        rollerTalon.set(TalonFXControlMode.PercentOutput, -0.8);
      }
      else if (shooterMode == ShooterMode.slow) {
        mainTalon.set(TalonFXControlMode.PercentOutput, 0.42);
        rollerTalon.set(TalonFXControlMode.PercentOutput, -0.42);
      }
      else if (shooterMode == ShooterMode.auto) {
        mainTalon.set(TalonFXControlMode.PercentOutput, 0.5);
        rollerTalon.set(TalonFXControlMode.PercentOutput, -0.8);
      }
      else {
        mainTalon.set(TalonFXControlMode.PercentOutput, 0);
        rollerTalon.set(TalonFXControlMode.PercentOutput, 0);
      }
      shooterReady = mainTalon.getMotorOutputPercent() != 0 || rollerTalon.getMotorOutputPercent() != 0;
    }
    else {
      if (shooterMode == ShooterMode.fast) {
        setVelocityMain = 2000;
        setVelocityRoller = -2000;
      }
      else if (shooterMode == ShooterMode.slow) {
        setVelocityMain = 1000;
        setVelocityRoller = -1000;
      }
      else if (shooterMode == ShooterMode.auto) {
        setVelocityMain = 2000;
        setVelocityRoller = -2000;
      }
      else {
        setVelocityMain = 0;
        setVelocityRoller = 0;
      }
      
      mainTalon.set(TalonFXControlMode.Velocity, setVelocityMain);
      rollerTalon.set(TalonFXControlMode.Velocity, setVelocityRoller);

      shooterReady = Math.abs(mainTalon.getSelectedSensorVelocity()-setVelocityMain) <= Constants.shooterDeadzone &&
                     Math.abs(rollerTalon.getSelectedSensorVelocity()-setVelocityRoller) <= Constants.shooterDeadzone;
    }

    SmartDashboard.putBoolean("Shooter Ready", shooterReady);
    SmartDashboard.putNumber("Shooter-Main", mainTalon.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter-Roller", rollerTalon.getSelectedSensorVelocity());
  }
}