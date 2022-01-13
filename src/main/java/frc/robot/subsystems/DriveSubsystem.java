// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  CANSparkMax m_frontLeftSpark = new CANSparkMax(Constants.frontLeftSparkID, MotorType.kBrushless);
  CANSparkMax m_frontRightSpark = new CANSparkMax(Constants.frontRightSparkID, MotorType.kBrushless);
  CANSparkMax m_backLeftSpark = new CANSparkMax(Constants.backLeftSparkID, MotorType.kBrushless);
  CANSparkMax m_backRightSpark = new CANSparkMax(Constants.backRightSparkID, MotorType.kBrushless);

  // Locations of the wheels relative to the robot center.
  Translation2d m_frontLeftLocation = new Translation2d(Constants.wheelDisFromCenter, Constants.wheelDisFromCenter);
  Translation2d m_frontRightLocation = new Translation2d(Constants.wheelDisFromCenter, -Constants.wheelDisFromCenter);
  Translation2d m_backLeftLocation = new Translation2d(-Constants.wheelDisFromCenter, Constants.wheelDisFromCenter);
  Translation2d m_backRightLocation = new Translation2d(-Constants.wheelDisFromCenter, -Constants.wheelDisFromCenter);

  // Creating my kinematics object using the wheel locations.
  MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  AHRS gyro;

  XboxController driveJoystick = new XboxController(0);
  Joystick driveJoystickMain = new Joystick(0);
  Joystick driveJoystickSide = new Joystick(1);

  public DriveSubsystem() {
    gyro = new AHRS(SPI.Port.kMXP);
  }

  @Override
  public void periodic() {
    if (gyro == null) {
      gyro = new AHRS(SPI.Port.kMXP);
      if (!gyro.isConnected()) gyro = null;
    }

    /** Creates a new DriveSubsystem. */
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(Constants.maxSpeed/driveJoystick.getX(Hand.kLeft),
                                                                Constants.maxSpeed/driveJoystick.getY(Hand.kLeft),
                                                                Constants.maxTurn/driveJoystick.getX(Hand.kRight),
                                                                Rotation2d.fromDegrees(gyro.getAngle()));

    // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(Constants.maxSpeed/driveJoystickMain.getX(),
    // Constants.maxSpeed/driveJoystickMain.getY(),
    // Constants.maxTurn/driveJoystickSide.getX(),
    // Rotation2d.fromDegrees(gyro.getAngle()));
    
    // Convert to wheel speeds
    MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    wheelSpeeds.normalize(Constants.maxSpeed);

    // Get the individual wheel speeds
    double frontLeft = wheelSpeeds.frontLeftMetersPerSecond;
    double frontRight = wheelSpeeds.frontRightMetersPerSecond;
    double backLeft = wheelSpeeds.rearLeftMetersPerSecond;
    double backRight = wheelSpeeds.rearRightMetersPerSecond;

    m_frontLeftSpark.set(Constants.maxSpeed/frontLeft);
    m_frontRightSpark.set(Constants.maxSpeed/frontRight);
    m_backLeftSpark.set(Constants.maxSpeed/backLeft);
    m_backRightSpark.set(Constants.maxSpeed/backRight);
  }
}
