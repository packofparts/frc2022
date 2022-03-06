// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  TalonFX climbFalconLeft = new TalonFX(IDs.climbTalonLeftID);
  TalonFX climbFalconRight = new TalonFX(IDs.climbTalonRighID);
  Joysticks joysticks;
  Solenoid climbSolenoid1;
  Solenoid climbSolenoid2;
  private final boolean usePneumatics = false;
     
  public ClimbSubsystem(Joysticks joysticks) {
    this.joysticks = joysticks; 
    if (usePneumatics) {
      climbSolenoid1 = new Solenoid(PneumaticsModuleType.REVPH, IDs.climbSolenoid1ID);
      climbSolenoid2 = new Solenoid(PneumaticsModuleType.REVPH, IDs.climbSolenoid2ID);
    }
  }

  @Override
  public void periodic() {
    climbFalconLeft.set(TalonFXControlMode.PercentOutput, joysticks.getClimbAxisLeft());
    climbFalconRight.set(TalonFXControlMode.PercentOutput, joysticks.getClimbAxisRight());
    
    if (usePneumatics) {
      if (joysticks.getIntakeSolenoidToggle()) {
        if (climbSolenoid1.get()) setPneumatics(false);
        else setPneumatics(true);
      }
    }
  }

  public void setPneumatics(boolean extend) {
    climbSolenoid1.set(extend);
    climbSolenoid2.set(extend);
  }
}
