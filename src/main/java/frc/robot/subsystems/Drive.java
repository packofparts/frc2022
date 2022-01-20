// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Gains;

public class Drive extends SubsystemBase {
  final double wheelDisFromCenter = 0.0;
  final double disFromBallErrorMargin = 1.0;
  
  CANSparkMax m_frontLeftSpark;
  CANSparkMax m_frontRightSpark;
  CANSparkMax m_backLeftSpark;
  CANSparkMax m_backRightSpark;

  // Locations of the wheels relative to the robot center.
  Translation2d m_frontLeftLocation = new Translation2d(Constants.wheelDisFromCenter, Constants.wheelDisFromCenter);
  Translation2d m_frontRightLocation = new Translation2d(Constants.wheelDisFromCenter, -Constants.wheelDisFromCenter);
  Translation2d m_backLeftLocation = new Translation2d(-Constants.wheelDisFromCenter, Constants.wheelDisFromCenter);
  Translation2d m_backRightLocation = new Translation2d(-Constants.wheelDisFromCenter, -Constants.wheelDisFromCenter);

  // Creating my kinematics object using the wheel locations.
  MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  AHRS gyro;
  Double gyroHold = null;

  XboxController driveJoystick = new XboxController(0);
  Joystick driveJoystickMain = new Joystick(0);
  Joystick driveJoystickSide = new Joystick(1);
  boolean usingXboxController = true;

  /** Creates a new Drive. */
  public Drive() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void drive(double power) {
    double xSpeed = 0;
    double ySpeed = 0;
    double rotation = 0;

    //controller inputs
    if (usingXboxController) {
      xSpeed = driveJoystick.getLeftX();
      ySpeed = -driveJoystick.getLeftY();
      rotation = driveJoystick.getRightX();
    }
    else {
      xSpeed = driveJoystickMain.getX();
      ySpeed = driveJoystickMain.getY();
      rotation = driveJoystickSide.getX();
    }

    //deadzone
    if (Math.abs(xSpeed) < Constants.joystickDeadzone) {
      xSpeed = 0;
    }
    if (Math.abs(ySpeed) < Constants.joystickDeadzone) {
      ySpeed = 0;
    }
    if (Math.abs(rotation) < Constants.joystickDeadzone) {
      rotation = 0;
    }

    //normalize
    if (xSpeed != 0) {
      xSpeed = Constants.maxSpeed / xSpeed;
    }
    if (ySpeed != 0) {
      ySpeed = Constants.maxSpeed / ySpeed;
    }
    if (rotation != 0) {
      rotation = Constants.maxTurnOutput / rotation;
      if (gyroHold != null) gyroHold = null;
    }
    //GYRO HOLD
    // else {
    //   if (gyroHold == null && gyro.getRate() < Constants.gyroDeadzone) {
    //     gyroHold = gyro.getAngle();
    //   }
    //   else {
    //     rotation = Constants.pGyro * (gyro.getAngle()-gyroHold);
    //     if (rotation > 1) rotation = 1;
    //     else if (rotation < -1) rotation = -1;
    //     else if (Math.abs(rotation) < Constants.minInput) rotation = 0;
    //   }
    // }

    // Convert to wheel speeds
    ChassisSpeeds speeds;
    if (usingXboxController) {
      if (driveJoystick.getXButton()) speeds = new ChassisSpeeds(xSpeed, ySpeed, rotation);
      else speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, Rotation2d.fromDegrees(gyro.getAngle()));
    }
    else {
      if (driveJoystickMain.getRawButton(1)) speeds = new ChassisSpeeds(xSpeed, ySpeed, rotation);
      else speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, Rotation2d.fromDegrees(gyro.getAngle()));
    }

    MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(Constants.maxSpeed);

    // Get the individual wheel speeds
    double frontLeft = wheelSpeeds.frontLeftMetersPerSecond;
    double frontRight = wheelSpeeds.frontRightMetersPerSecond;
    double backLeft = wheelSpeeds.rearLeftMetersPerSecond;
    double backRight = wheelSpeeds.rearRightMetersPerSecond;

    m_frontLeftSpark.set(frontLeft/Constants.maxSpeed);
    m_frontRightSpark.set(frontRight/Constants.maxSpeed);
    m_backLeftSpark.set(backLeft/Constants.maxSpeed);
    m_backRightSpark.set(backRight/Constants.maxSpeed);
    
    // m_frontLeftSpark.getPIDController().setReference(frontLeft, ControlType.kSmartVelocity);
    // m_frontRightSpark.getPIDController().setReference(frontRight, ControlType.kSmartVelocity);
    // m_backLeftSpark.getPIDController().setReference(backLeft, ControlType.kSmartVelocity);
    // m_backRightSpark.getPIDController().setReference(backRight, ControlType.kSmartVelocity);

    SmartDashboard.putNumber("frontLeftPos", m_frontLeftSpark.getEncoder().getPosition());
    SmartDashboard.putNumber("frontRightPos", m_frontRightSpark.getEncoder().getPosition());
    SmartDashboard.putNumber("rearLeftPos", m_backLeftSpark.getEncoder().getPosition());
    SmartDashboard.putNumber("rearRightPos", m_backRightSpark.getEncoder().getPosition());
  }

  public void setRampRates(double time) {
    m_frontLeftSpark.setOpenLoopRampRate(time);
    m_frontLeftSpark.setClosedLoopRampRate(time);

    m_frontRightSpark.setOpenLoopRampRate(time);
    m_frontRightSpark.setClosedLoopRampRate(time);

    m_backLeftSpark.setOpenLoopRampRate(time);
    m_backLeftSpark.setClosedLoopRampRate(time);

    m_backRightSpark.setOpenLoopRampRate(time);
    m_backRightSpark.setClosedLoopRampRate(time);
  }

  public double yaw() {
    return gyro.getAngle();
  }
}
