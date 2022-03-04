// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.stream.DoubleStream;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.Gains;
import frc.robot.constants.Constants;
import frc.robot.constants.IDs;

public class DriveSubsystem extends SubsystemBase {
  public CANSparkMax m_frontLeftSpark;
  public CANSparkMax m_frontRightSpark;
  public CANSparkMax m_backLeftSpark;
  public CANSparkMax m_backRightSpark;

  // Locations of the wheels relative to the robot center.
  Translation2d m_frontLeftLocation = new Translation2d(Constants.wheelDisFromCenter, Constants.wheelDisFromCenter);
  Translation2d m_frontRightLocation = new Translation2d(Constants.wheelDisFromCenter, -Constants.wheelDisFromCenter);
  Translation2d m_backLeftLocation = new Translation2d(-Constants.wheelDisFromCenter, Constants.wheelDisFromCenter);
  Translation2d m_backRightLocation = new Translation2d(-Constants.wheelDisFromCenter, -Constants.wheelDisFromCenter);

  // Creating my kinematics object using the wheel locations.
  MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  String bruh = "Ask me who Joe is";
  AHRS initGyro;
  AHRS gyro;
  Double gyroHold = null;
  boolean shouldDrive = true;

  PIDController turnPID = new PIDController(Constants.turnPID[0], Constants.turnPID[1], Constants.turnPID[2]);
  PIDController ratePID = new PIDController(Constants.ratePID[0], Constants.ratePID[1], Constants.ratePID[2]);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  Joysticks joysticks;
  
  //set booleans
  final boolean useGyroHold = true;
  final boolean usingXboxController = false;//Joysticks.driveXboxController != null;
  final boolean tuningPID = false;

  public DriveSubsystem(Joysticks joysticks) {
    /*
sHAKUANDO WAS HERE
    */
    CameraServer.getInstance().startAutomaticCapture();
    CameraServer.getInstance().startAutomaticCapture();

    this.joysticks = joysticks;
    initGyro = new AHRS(SPI.Port.kMXP);

    m_frontLeftSpark = new CANSparkMax(IDs.frontLeftSparkID, MotorType.kBrushless);
    m_frontRightSpark = new CANSparkMax(IDs.frontRightSparkID, MotorType.kBrushless);
    m_backLeftSpark = new CANSparkMax(IDs.backLeftSparkID, MotorType.kBrushless);
    m_backRightSpark = new CANSparkMax(IDs.backRightSparkID, MotorType.kBrushless);
    m_frontLeftSpark.restoreFactoryDefaults(true);
    m_frontRightSpark.restoreFactoryDefaults(true);
    m_backLeftSpark.restoreFactoryDefaults(true);
    m_backRightSpark.restoreFactoryDefaults(true);

    m_frontLeftSpark.getEncoder();
    m_frontRightSpark.getEncoder();
    m_backLeftSpark.getEncoder();
    m_backRightSpark.getEncoder();

    m_frontLeftSpark.getEncoder().setPosition(0);
    m_frontRightSpark.getEncoder().setPosition(0);
    m_backLeftSpark.getEncoder().setPosition(0);
    m_backRightSpark.getEncoder().setPosition(0);

    m_frontLeftSpark.getEncoder().setPositionConversionFactor(Constants.encoderConversion);
    m_frontRightSpark.getEncoder().setPositionConversionFactor(Constants.encoderConversion);
    m_backLeftSpark.getEncoder().setPositionConversionFactor(Constants.encoderConversion);
    m_backRightSpark.getEncoder().setPositionConversionFactor(Constants.encoderConversion);

    m_frontLeftSpark.setSmartCurrentLimit(60);
    m_frontRightSpark.setSmartCurrentLimit(60);
    m_backLeftSpark.setSmartCurrentLimit(60);
    m_backRightSpark.setSmartCurrentLimit(60);

    setRampRates(0.5);
    setMode(idleMode.brake);

    setAllPIDControllers(Constants.movePID);
    setAllPIDControllers(Constants.velocityPID);

    m_frontLeftSpark.setInverted(false);
    m_frontRightSpark.setInverted(true);
    m_backLeftSpark.setInverted(false);
    m_backRightSpark.setInverted(true);
  }

  @Override
  public void periodic() {
    if (gyro == null) {
      if (initGyro.isConnected()) gyro = initGyro;
    }
    else {
      if (gyro.isCalibrating()) {
        SmartDashboard.putString("Gyro Yaw", "Calibrating");
        SmartDashboard.putString("Gyro Rate", "Calibrating");
      }
      else {
        SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
        SmartDashboard.putNumber("Gyro Rate", gyro.getRate());
      }
    }
    
    if (joysticks.getGyroResetButton()) resetGyro();
    if (joysticks.getEncoderResetButton()) resetEncoders();

    if (tuningPID) {
      //modify PID w/ joysticks
      if (joysticks.getPIncrease()) turnPID.setP(turnPID.getP() + 0.0005);;
      if (joysticks.getPDecrease()) turnPID.setP(turnPID.getP() - 0.0005);
      if (joysticks.getIIncrease()) turnPID.setI(turnPID.getI() + 0.0000005);
      if (joysticks.getIDecrease()) turnPID.setI(turnPID.getI() - 0.0000005);
      if (joysticks.getDIncrease()) turnPID.setD(turnPID.getD() + 0.00005);
      if (joysticks.getDDecrease()) turnPID.setD(turnPID.getD() - 0.00005);

      //display PID values
      SmartDashboard.putNumber("P: ", turnPID.getP());
      SmartDashboard.putNumber("I: ", turnPID.getI());
      SmartDashboard.putNumber("D: ", turnPID.getD());

      //calculate PID on button input
      Double pidOutput = null;
      if (joysticks.getPIDLeft()) pidOutput = -turnPID.calculate(gyro.getAngle(), 90);
      else if (joysticks.getPIDRight()) pidOutput = -turnPID.calculate(gyro.getAngle(), -90);

      //apply PID on button input
      if (pidOutput != null) drive(0, 0, pidOutput, true);
      else drive(0, 0, 0, true);
      SmartDashboard.putString("pid output", pidOutput+"");
    }
    else drive();
  }

  //drive with joystick inputs
  public void drive() {
    if (!shouldDrive) return;

    double xSpeed = 0;
    double ySpeed = 0;
    double rotation = 0;

    //controller inputs
    xSpeed = -joysticks.getDriveSideways();
    ySpeed = -joysticks.getDriveForward();
    rotation = -joysticks.getDriveRotation();
    
    //deadzone
    if (Math.abs(xSpeed) < Constants.joystickDeadzone) xSpeed = 0;
    if (Math.abs(ySpeed) < Constants.joystickDeadzone) ySpeed = 0;
    if (Math.abs(rotation) < Constants.joystickDeadzone) rotation = 0;

    //apply input scaling
    xSpeed = ((1-Constants.minPower) * (Math.pow(((Math.abs(xSpeed)-Constants.joystickDeadzone) * (1/(1-Constants.joystickDeadzone))), Constants.exponentFactor)) + Constants.minPower) * getSign(xSpeed);
    ySpeed = ((1-Constants.minPower) * (Math.pow(((Math.abs(ySpeed)-Constants.joystickDeadzone) * (1/(1-Constants.joystickDeadzone))), Constants.exponentFactor)) + Constants.minPower) * getSign(ySpeed);
    rotation = ((1-Constants.minTurn) * (Math.pow(((Math.abs(rotation)-Constants.joystickDeadzone) * (1/(1-Constants.joystickDeadzone))), Constants.turnExponentFactor)) + Constants.minTurn) * getSign(rotation);

    //precision toggle
    if (joysticks.getPrecisionRotationToggle()) rotation *= Constants.precisionFactor;

    //field-oriented toggle
    boolean fieldOrientated = true;
    if (joysticks.getRobotOrientedToggle()) fieldOrientated = false;

    drive(xSpeed, ySpeed, rotation, fieldOrientated);
  }

  //raw drive method used for external commands
  public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldOriented) {
    //normalize
    if (xSpeed != 0) xSpeed = Constants.maxSpeed * xSpeed;
    if (ySpeed != 0) ySpeed = Constants.maxSpeed * ySpeed;
    if (rotation != 0) {
      if (gyroHold != null) gyroHold = null;
    }
    //GYRO HOLD
    else {
      if (useGyroHold) {
        //set gyroHold angle
        if (gyroHold == null) gyroHold = gyro.getAngle();
        //activate gyroHold PID
        if (gyroHold != null) rotation = -turnPID.calculate(gyro.getAngle(), gyroHold) * Constants.rateAggresiveness;
      }
    }
    //closed loop turning
    rotation = -ratePID.calculate(gyro.getRate(), rotation*Constants.rateFactor*Constants.maxRate) * Constants.maxTurnOutput;
    // rotation *= Constants.maxTurnOutput;
    SmartDashboard.putString("gyrohold", gyroHold+"");
    SmartDashboard.putNumber("rotation", rotation);

    // Convert to wheel speeds
    ChassisSpeeds speeds;
    if (!fieldOriented) speeds = new ChassisSpeeds(ySpeed, xSpeed, rotation);
    else speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rotation, Rotation2d.fromDegrees(-gyro.getAngle()));

    MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    double[] wheelSpeedDouble = new double[] {wheelSpeeds.frontLeftMetersPerSecond,
      wheelSpeeds.frontRightMetersPerSecond,
      wheelSpeeds.rearLeftMetersPerSecond,
      wheelSpeeds.rearRightMetersPerSecond};
    desaturate(Constants.maxSpeed, wheelSpeedDouble);

    // Get the individual wheel speeds
    double frontLeft = wheelSpeedDouble[0];
    double frontRight = wheelSpeedDouble[1];
    double backLeft = wheelSpeedDouble[2];
    double backRight = wheelSpeedDouble[3];

    //set motor speeds
    m_frontLeftSpark.getPIDController().setReference(frontLeft, ControlType.kVelocity);
    m_frontRightSpark.getPIDController().setReference(frontRight, ControlType.kVelocity);
    m_backLeftSpark.getPIDController().setReference(backLeft, ControlType.kVelocity);
    m_backRightSpark.getPIDController().setReference(backRight, ControlType.kVelocity);

    // SmartDashboard.putNumber("frontLeftSet", frontLeft);
    // SmartDashboard.putNumber("frontRightSet", frontRight);
    // SmartDashboard.putNumber("rearLeftSet", backLeft);
    // SmartDashboard.putNumber("rearRightSet", backRight);

    // SmartDashboard.putNumber("frontLeftPos", m_frontLeftSpark.getEncoder().getPosition());
    // SmartDashboard.putNumber("frontRightPos", m_frontRightSpark.getEncoder().getPosition());
    // SmartDashboard.putNumber("rearLeftPos", m_backLeftSpark.getEncoder().getPosition());
    // SmartDashboard.putNumber("rearRightPos", m_backRightSpark.getEncoder().getPosition());
    
    // SmartDashboard.putNumber("frontLeftVel", m_frontLeftSpark.getEncoder().getVelocity());
    // SmartDashboard.putNumber("frontRightVel", m_frontRightSpark.getEncoder().getVelocity());
    // SmartDashboard.putNumber("rearLeftVel", m_backLeftSpark.getEncoder().getVelocity());
    // SmartDashboard.putNumber("rearRightVel", m_backRightSpark.getEncoder().getVelocity());
  }

  public void resetEncoders() {
    m_frontLeftSpark.getEncoder().setPosition(0);
    m_frontRightSpark.getEncoder().setPosition(0);
    m_backLeftSpark.getEncoder().setPosition(0);
    m_backRightSpark.getEncoder().setPosition(0);
  }

  public void resetGyro() {
    if (gyro != null && gyro.isConnected() && !gyro.isCalibrating()) {
      gyro.reset();
      this.gyroHold = 0.0;
    }
  }

  public void setShouldDrive(boolean drive) {
    this.shouldDrive = drive;
  }

  public int getSign(double val) {
    if (val > 0) return 1;
    else if (val < 0) return -1;
    return 0;
  }

  public void desaturate(double attainableMaxSpeedMetersPerSecond, double[] speeds) {
    double realMaxSpeed = DoubleStream.of(speeds).max().getAsDouble();
    if (Math.abs(DoubleStream.of(speeds).min().getAsDouble()) > Math.abs(realMaxSpeed)) {
      realMaxSpeed = DoubleStream.of(speeds).min().getAsDouble();
    }

    if (Math.abs(realMaxSpeed) > attainableMaxSpeedMetersPerSecond) {
      speeds[0] = speeds[0] / Math.abs(realMaxSpeed) * attainableMaxSpeedMetersPerSecond;
      speeds[1] = speeds[1] / Math.abs(realMaxSpeed) * attainableMaxSpeedMetersPerSecond;
      speeds[2] = speeds[2] / Math.abs(realMaxSpeed) * attainableMaxSpeedMetersPerSecond;
      speeds[3] = speeds[3] / Math.abs(realMaxSpeed) * attainableMaxSpeedMetersPerSecond;
    }
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
  
  private void setAllPIDControllers(Gains pidSet) {
    setPidControllers(m_frontLeftSpark.getPIDController(), pidSet, pidSet.kSlot);
    setPidControllers(m_frontRightSpark.getPIDController(), pidSet, pidSet.kSlot);
    setPidControllers(m_backLeftSpark.getPIDController(), pidSet, pidSet.kSlot);
    setPidControllers(m_backRightSpark.getPIDController(), pidSet, pidSet.kSlot);
  }

  private void setPidControllers (SparkMaxPIDController pidController, Gains pidSet, int slot) {
    pidController.setP(pidSet.kP, slot);
    pidController.setI(pidSet.kI, slot);
    pidController.setD(pidSet.kD, slot);
    pidController.setIZone(pidSet.kIz, slot);
    pidController.setFF(pidSet.kFF, slot);
    pidController.setOutputRange(pidSet.kMinOutput, pidSet.kMaxOutput, slot);
  }

  public void setFrontLeftPosition(double pos) {
    m_frontLeftSpark.getPIDController().setReference(pos, ControlType.kPosition, Constants.movePID.kSlot);
  }
  public void setBackLeftPosition(double pos) {
    m_backLeftSpark.getPIDController().setReference(pos, ControlType.kPosition, Constants.movePID.kSlot);
  }
  public void setFrontRightPosition(double pos) {
    m_frontRightSpark.getPIDController().setReference(pos, ControlType.kPosition, Constants.movePID.kSlot);
  }
  public void setBackRightPosition(double pos) {
    m_backRightSpark.getPIDController().setReference(pos, ControlType.kPosition, Constants.movePID.kSlot);
  }
  
  public void setFrontLeftVelocity(double vel) {
    m_frontLeftSpark.getPIDController().setReference(vel, ControlType.kVelocity, Constants.velocityPID.kSlot);
  }
  public void setBackLeftVelocity(double vel) {
    m_backLeftSpark.getPIDController().setReference(vel, ControlType.kVelocity, Constants.velocityPID.kSlot);
  }
  public void setFrontRightVelocity(double vel) {
    m_frontRightSpark.getPIDController().setReference(vel, ControlType.kVelocity, Constants.velocityPID.kSlot);
  }
  public void setBackRightVelocity(double vel) {
    m_backRightSpark.getPIDController().setReference(vel, ControlType.kVelocity, Constants.velocityPID.kSlot);
  }

  public void stop() {
    m_frontLeftSpark.set(0);
    m_frontRightSpark.set(0);
    m_backLeftSpark.set(0);
    m_backRightSpark.set(0);
  }

  public double getAveragePos() {
    return (getFrontLeftPos()+getFrontRightPos()+getRearLeftPos()+getRearRightPos())/4;
  }

  public double getFrontLeftPos() {
    return m_frontLeftSpark.getEncoder().getPosition();
  }
  public double getFrontRightPos() {
    return m_frontRightSpark.getEncoder().getPosition();
  }
  public double getRearLeftPos() {
    return m_backLeftSpark.getEncoder().getPosition();
  }
  public double getRearRightPos() {
    return m_backRightSpark.getEncoder().getPosition();
  }

  public double getAngle() {
    return gyro.getAngle();
  }
}
