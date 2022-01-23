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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Gains;

public class DriveSubsystem extends SubsystemBase {
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

  //set booleans
  final boolean useGyroHold = false;
  final boolean usingXboxController = false;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  
  public DriveSubsystem() {
    gyro = new AHRS(SPI.Port.kMXP); 

    m_frontLeftSpark = new CANSparkMax(Constants.frontLeftSparkID, MotorType.kBrushless);
    m_frontRightSpark = new CANSparkMax(Constants.frontRightSparkID, MotorType.kBrushless);
    m_backLeftSpark = new CANSparkMax(Constants.backLeftSparkID, MotorType.kBrushless);
    m_backRightSpark = new CANSparkMax(Constants.backRightSparkID, MotorType.kBrushless);

    m_frontLeftSpark.restoreFactoryDefaults(true);
    m_frontRightSpark.restoreFactoryDefaults(true);
    m_backLeftSpark.restoreFactoryDefaults(true);
    m_backRightSpark.restoreFactoryDefaults(true);

    m_frontLeftSpark.getEncoder();
    m_frontRightSpark.getEncoder();
    m_backLeftSpark.getEncoder();
    m_backRightSpark.getEncoder();

    m_frontLeftSpark.setSmartCurrentLimit(60);
    m_frontRightSpark.setSmartCurrentLimit(60);
    m_backLeftSpark.setSmartCurrentLimit(60);
    m_backRightSpark.setSmartCurrentLimit(60);

    setRampRates(0.5);
    setMode(idleMode.coast);

    setPidControllers(m_frontLeftSpark.getPIDController(), Constants.defaultPID, Constants.defaultPID.kSlot);
    setPidControllers(m_frontRightSpark.getPIDController(), Constants.defaultPID, Constants.defaultPID.kSlot);
    setPidControllers(m_backLeftSpark.getPIDController(), Constants.defaultPID, Constants.defaultPID.kSlot);
    setPidControllers(m_backRightSpark.getPIDController(), Constants.defaultPID, Constants.defaultPID.kSlot);

    m_frontLeftSpark.setInverted(false);
    m_frontRightSpark.setInverted(true);
    m_backLeftSpark.setInverted(false);
    m_backRightSpark.setInverted(true);
  }

  @Override
  public void periodic() {
    if (gyro == null) {
      gyro = new AHRS(SPI.Port.kMXP);
      if (!gyro.isConnected()) gyro = null;
    }
    else {
      SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
      SmartDashboard.putNumber("Gyro Rate", gyro.getRate());
    }

    if (usingXboxController) {
      if (driveJoystick.getRawButton(9)) gyro.reset();
    }
    else {
      if (driveJoystickMain.getRawButton(8)) gyro.reset();
    }

    m_frontLeftSpark.getPIDController().setReference(-driveJoystickMain.getY() * Constants.maxSpeed, ControlType.kVelocity);
    SmartDashboard.putNumber("frontLeftSet", -driveJoystickMain.getY() * Constants.maxSpeed);

    SmartDashboard.putNumber("frontLeftVel", m_frontLeftSpark.getEncoder().getVelocity());
    SmartDashboard.putNumber("frontRightVel", m_frontRightSpark.getEncoder().getVelocity());
    SmartDashboard.putNumber("rearLeftVel", m_backLeftSpark.getEncoder().getVelocity());
    SmartDashboard.putNumber("rearRightVel", m_backRightSpark.getEncoder().getVelocity());

    // drive();
  }

  public void drive() {
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
      ySpeed = -driveJoystickMain.getY();
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

    // double turnFactor = (1-fastestTurn) * Math.pow(1-Math.pow(fastestTurn, 8/3), 6) + minTurn;
    // xSpeed = ((1-minPower) * Math.abs(Math.pow(xSpeed, 8/3)) + minPower) * getSign(xSpeed);
    // ySpeed = ((1-minPower) * Math.abs(Math.pow(ySpeed, 8/3)) + minPower) * getSign(ySpeed);
    // rotation = ((1-minTurn) * Math.abs(Math.pow(turn, 8/3)) + minTurn) * getSign(turn) * turnFactor;

    //normalize
    if (xSpeed != 0) {
      xSpeed = Constants.maxSpeed * xSpeed;
    }
    if (ySpeed != 0) {
      ySpeed = Constants.maxSpeed * ySpeed;
    }
    if (rotation != 0) {
      rotation = Constants.maxTurnOutput * rotation;
      if (gyroHold != null) gyroHold = null;
    }
    //GYRO HOLD
    else {
      if (!useGyroHold) gyroHold = null;

      if (gyroHold == null && gyro.getRate() < Constants.gyroDeadzone) {
        gyroHold = gyro.getAngle();
      }
      else {
        rotation = Constants.pGyro * (gyro.getAngle()-gyroHold);
        if (rotation > 1) rotation = 1;
        else if (rotation < -1) rotation = -1;
        else if (Math.abs(rotation) < Constants.minInput) rotation = 0;
      }
    }

    // Convert to wheel speeds
    ChassisSpeeds speeds;
    if (usingXboxController) {
      if (driveJoystick.getXButton()) speeds = new ChassisSpeeds(ySpeed, xSpeed, rotation);
      else speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rotation, Rotation2d.fromDegrees(gyro.getAngle()));
    }
    else {
      if (driveJoystickMain.getRawButton(1)) speeds = new ChassisSpeeds(ySpeed, xSpeed, rotation);
      else speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rotation, Rotation2d.fromDegrees(gyro.getAngle()));
    }

    MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(Constants.maxSpeed);

    // Get the individual wheel speeds
    double frontLeft = wheelSpeeds.frontLeftMetersPerSecond;
    double frontRight = wheelSpeeds.frontRightMetersPerSecond;
    double backLeft = wheelSpeeds.rearLeftMetersPerSecond;
    double backRight = wheelSpeeds.rearRightMetersPerSecond;

    //set motor speeds
    m_frontLeftSpark.set(frontLeft/Constants.maxSpeed);
    m_frontRightSpark.set(frontRight/Constants.maxSpeed);
    m_backLeftSpark.set(backLeft/Constants.maxSpeed);
    m_backRightSpark.set(backRight/Constants.maxSpeed);

    // m_frontLeftSpark.getPIDController().setReference(frontLeft, ControlType.kSmartVelocity);
    // m_frontRightSpark.getPIDController().setReference(frontRight, ControlType.kSmartVelocity);
    // m_backLeftSpark.getPIDController().setReference(backLeft, ControlType.kSmartVelocity);
    // m_backRightSpark.getPIDController().setReference(backRight, ControlType.kSmartVelocity);

    SmartDashboard.putNumber("frontLeftSet", m_frontLeftSpark.get());
    SmartDashboard.putNumber("frontRightSet", m_frontRightSpark.get());
    SmartDashboard.putNumber("rearLeftSet", m_backLeftSpark.get());
    SmartDashboard.putNumber("rearRightSet", m_backRightSpark.get());

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

  private enum idleMode {
    brake,
    coast
  }

  public void setMode(idleMode type) {
    if (type == idleMode.brake) {
      m_frontLeftSpark.setIdleMode(IdleMode.kBrake);
      m_frontRightSpark.setIdleMode(IdleMode.kBrake);
      m_backLeftSpark.setIdleMode(IdleMode.kBrake);
      m_backRightSpark.setIdleMode(IdleMode.kBrake);
    } else if (type == idleMode.coast) {
      m_frontLeftSpark.setIdleMode(IdleMode.kCoast);
      m_frontRightSpark.setIdleMode(IdleMode.kCoast);
      m_backLeftSpark.setIdleMode(IdleMode.kCoast);
      m_backRightSpark.setIdleMode(IdleMode.kCoast);
    }
  }

  private void setPidControllers (SparkMaxPIDController pidController, Gains pidSet, int slot) {
    pidController.setP(pidSet.kP, slot);
    pidController.setI(pidSet.kI, slot);
    pidController.setD(pidSet.kD, slot);
    pidController.setIZone(pidSet.kIz, slot);
    pidController.setFF(pidSet.kFF, slot);
    pidController.setOutputRange(pidSet.kMinOutput, pidSet.kMaxOutput, slot);
  }
}
