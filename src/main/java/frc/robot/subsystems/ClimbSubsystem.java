// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IDs;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  TalonFX climbFalcon = new TalonFX(IDs.climbTalonID);
  Joysticks joysticks;

  public ClimbSubsystem(Joysticks joysticks) {
    this.joysticks = joysticks;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climbFalcon.set(TalonFXControlMode.PercentOutput, joysticks.getClimbAxis());
    SmartDashboard.putNumber("Climb Pos", climbFalcon.getSelectedSensorPosition());
  }

  public double getPos() {
    return climbFalcon.getSelectedSensorPosition() * Constants.climbFactor;
  }
}
