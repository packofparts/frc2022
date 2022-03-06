// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Blob;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class Joysticks {
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
  public boolean getPIDLeft() {return driveJoystickSide.getRawButton(4);}
  public boolean getPIDRight() {return driveJoystickSide.getRawButton(5);}


  //operator controls
  private final XboxController operatorController = new XboxController(2);
  public double getClimbAxisRight() {return operatorController.getRightY();}
  public double getClimbAxisLeft() {return operatorController.getLeftY();}
  public double getClimbAxis() {return operatorController.getLeftTriggerAxis()-operatorController.getRightTriggerAxis();}
  public boolean getClimbSolenoidToggle() {return operatorController.getYButtonPressed();}

  public boolean getIntakeSolenoidToggle() {return operatorController.getXButtonPressed();}
  public boolean getIndexToggle() {return operatorController.getBButton();}
  // public boolean getOutdexToggle() {return operatorController.getXButton();}
  public boolean getIntakeToggle() {return operatorController.getRightBumper();}
  public boolean getOutakeToggle() {return operatorController.getLeftBumper();}
  public boolean getIncreaseShooter() {return operatorController.getPOV() == 0;}
  public boolean getDecreaseShooter() {return operatorController.getPOV() == 180;}
  // switch to is pressed
  public boolean getNotPOV() {return operatorController.getPOV() == -1;}
  public boolean getShooterScrollRight() {return operatorController.getPOV() == 90;}
  public boolean getShooterScrollLeft() {return operatorController.getPOV() == 270;}
  public boolean getShooterScrollDown() {return operatorController.getPOV() == 180;}
  public boolean getShooterToggle() {return operatorController.getAButtonPressed();}
}
