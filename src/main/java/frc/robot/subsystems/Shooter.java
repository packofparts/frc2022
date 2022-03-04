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

  boolean tuningRPM = false;
  
  public Shooter(Joysticks joysticks) {
    this.joysticks = joysticks;

    mainTalon.config_kP(0, Constants.shooterMainPID[0]);
    mainTalon.config_kI(0, Constants.shooterMainPID[1]);
    mainTalon.config_kD(0, Constants.shooterMainPID[2]);
    
    rollerTalon.config_kP(0, Constants.shooterRollerPID[0]);
    rollerTalon.config_kI(0, Constants.shooterRollerPID[1]);
    rollerTalon.config_kD(0, Constants.shooterRollerPID[2]);

    if (tuningRPM) {
      SmartDashboard.putNumber("mainRoll", 0);
      SmartDashboard.putNumber("secondRoll", 0); 
    }
  }

  public enum ShooterMode {
    launchPad, tarmac, auto, off;
  }

  @Override
  public void periodic() {
    //shooter toggle
    if (joysticks.getShooterHigh()) {
      if (shooterMode != ShooterMode.off) setShooterMode(ShooterMode.off);
      else setShooterMode(ShooterMode.launchPad);
    }
    else if (joysticks.getShooterLow()) {
      if (shooterMode != ShooterMode.off) setShooterMode(ShooterMode.off);
      else setShooterMode(ShooterMode.tarmac);
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
    if (tuningRPM) {
      setVelocityMain = SmartDashboard.getNumber("mainRoll", 0);
      setVelocityRoller = SmartDashboard.getNumber("secondRoll", 0);
    }
    else {
      if (shooterMode == ShooterMode.launchPad) {
        setVelocityMain = 6600;
        setVelocityRoller = -4500;
      }
      else if (shooterMode == ShooterMode.tarmac) {
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
    }
    
    if (setVelocityMain == 0) mainTalon.set(TalonFXControlMode.PercentOutput, 0);
    else mainTalon.set(TalonFXControlMode.Velocity, setVelocityMain);

    if (setVelocityRoller == 0) rollerTalon.set(TalonFXControlMode.PercentOutput, 0);
    else rollerTalon.set(TalonFXControlMode.Velocity, setVelocityRoller);

    shooterReady = Math.abs(Math.abs(setVelocityMain)-Math.abs(mainTalon.getSelectedSensorVelocity())) <= Constants.shooterDeadzone &&
    Math.abs(Math.abs(setVelocityRoller)-Math.abs(rollerTalon.getSelectedSensorVelocity())) <= Constants.shooterDeadzone && 
                    setVelocityMain != 0 && setVelocityRoller != 0;

    SmartDashboard.putBoolean("Shooter Running", mainTalon.getMotorOutputPercent() != 0 || rollerTalon.getMotorOutputPercent() != 0);
    SmartDashboard.putBoolean("Shooter Ready", shooterReady);
    SmartDashboard.putNumber("Shooter-Main", mainTalon.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter-Roller", rollerTalon.getSelectedSensorVelocity());
  }
}