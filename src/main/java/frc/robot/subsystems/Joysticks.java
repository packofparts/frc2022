// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Joysticks extends SubsystemBase {
  // private final XboxController driveXboxController = null;//new XboxController(0);
  // public final int robotOrientedToggle = XboxController.Button.kA.value;

  //main drive joystick
  private final Joystick driveJoystickMain = new Joystick(0);
  public double getDriveForward() {return driveJoystickMain.getY();}
  public double getDriveSideways() {return driveJoystickMain.getX();}

  public boolean getRobotOrientedToggle() {return driveJoystickMain.getRawButton(1);}
  public boolean getGyroResetButton() {return driveJoystickMain.getRawButtonPressed(8);}
  public boolean getEncoderResetButton() {return driveJoystickMain.getRawButtonPressed(9);}


  //side drive joysticks
  private final Joystick driveJoystickSide = new Joystick(1);
  public double getDriveRotation() {return driveJoystickSide.getX();}
  public boolean getPrecisionRotationToggle() {return driveJoystickSide.getRawButton(3);}

  public boolean getPIncrease() {return driveJoystickSide.getRawButtonReleased(6);}
  public boolean getPDecrease() {return driveJoystickSide.getRawButtonReleased(7);}
  public boolean getIIncrease() {return driveJoystickSide.getRawButtonReleased(8);}
  public boolean getIDecrease() {return driveJoystickSide.getRawButtonReleased(9);}
  public boolean getDIncrease() {return driveJoystickSide.getRawButtonReleased(11);}
  public boolean getDDecrease() {return driveJoystickSide.getRawButtonReleased(10);}
  public boolean getPIDSlow() {return driveJoystickSide.getRawButton(4);}
  public boolean getPIDFast() {return driveJoystickSide.getRawButton(5);}


  //operator controls
  private final XboxController operatorController = new XboxController(2);
  public double getClimbAxis() {return operatorController.getLeftY();}

  public boolean getIntakeSolenoidToggle() {return operatorController.getBButton();}
  public boolean getIndexToggle() {return operatorController.getBButton();}
  public boolean getOutdexToggle() {return operatorController.getXButton();}
  public boolean getIntakeToggle() {return operatorController.getRightBumper();}
  public boolean getOutakeToggle() {return operatorController.getLeftBumper();}
  public boolean getIncreaseShooter() {return operatorController.getPOV() == 0;}
  public boolean getDecreaseShooter() {return operatorController.getPOV() == 180;}

  
  public double getShooterMain() {return operatorController.getLeftY();}
  public double getShooterRoller() {return operatorController.getRightY();}
  public boolean getShooterHigh() {return operatorController.getYButtonPressed();}
  public boolean getShooterLow() {return operatorController.getAButtonPressed();}
}
