// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  TalonFX climbFalconLeft = new TalonFX(IDs.climbTalonLeftID);
  TalonFX climbFalconRight = new TalonFX(IDs.climbTalonRighID);
  Joysticks joysticks;
  Solenoid climbSolenoid1;
  Solenoid climbSolenoid2;
  
  private final boolean usePneumatics = true;
  private boolean manualMode = false;
     
  public ClimbSubsystem(Joysticks joysticks) {
    this.joysticks = joysticks; 
    if (usePneumatics) {
      climbSolenoid1 = new Solenoid(PneumaticsModuleType.REVPH, IDs.climbSolenoid1ID);
      climbSolenoid2 = new Solenoid(PneumaticsModuleType.REVPH, IDs.climbSolenoid2ID);
    }
    climbFalconLeft.setSelectedSensorPosition(0);
    climbFalconRight.setSelectedSensorPosition(0);
    climbFalconLeft.setNeutralMode(NeutralMode.Brake);
    climbFalconLeft.setInverted(true);
    climbFalconRight.setNeutralMode(NeutralMode.Brake);

    SmartDashboard.putBoolean("Manual Climb", false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("leftClimbPos", climbFalconLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("rightClimbPos", climbFalconRight.getSelectedSensorPosition());

    //ignore climb limits, and seperate joystick control
    manualMode = SmartDashboard.getBoolean("Manual Climb", false);

    //climb limits
    double climbLeft = 0.0;
    boolean leftControl = false;
    //ignore lower limit during manual mode
    if ((climbFalconLeft.getSelectedSensorPosition() <= -2000 || manualMode) && climbFalconLeft.getSelectedSensorPosition() >= -250000 && joysticks.getClimbAxisLeftPressed()) leftControl = true;
    else if (climbFalconLeft.getSelectedSensorPosition() < -250000) climbLeft = 0.2;
    else if (climbFalconLeft.getSelectedSensorPosition() > -2000 && !manualMode) climbLeft = -0.2;

    double climbRight = 0.0;
    boolean rightControl = false;
    //ignore lower limit during manual mode
    if ((climbFalconRight.getSelectedSensorPosition() <= -2000 || manualMode) && climbFalconRight.getSelectedSensorPosition() >= -250000 && joysticks.getClimbAxisRightPressed()) rightControl = true;
    else if (climbFalconRight.getSelectedSensorPosition() < -250000) climbRight = 0.2;
    else if (climbFalconRight.getSelectedSensorPosition() > -2000 && !manualMode) climbRight = -0.2;

    //joystick controls
    //seperate axis in manual mode
    if (manualMode) {
      if (leftControl) climbLeft = joysticks.getClimbAxisLeft();
      if (rightControl) climbRight = joysticks.getClimbAxisRight();
    }
    //otherwise combined axis
    else {
      if (leftControl) climbLeft = joysticks.getClimbAxisLeft();
      if (rightControl) climbRight = joysticks.getClimbAxisLeft();
    }

    climbFalconLeft.set(TalonFXControlMode.PercentOutput, climbLeft);
    climbFalconRight.set(TalonFXControlMode.PercentOutput, climbRight);
    
    if (joysticks.getClimbReset()) {
      climbFalconLeft.setSelectedSensorPosition(0);
      climbFalconRight.setSelectedSensorPosition(0);
    }
    if (usePneumatics) {
      if (joysticks.getClimbSolenoidToggle()) {
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
