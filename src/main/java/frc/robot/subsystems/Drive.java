// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Gains;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.MoveBy;

public class Drive extends SubsystemBase {
  public final CANSparkMax m_frontLeftSpark = new CANSparkMax(Constants.frontLeftSparkID, MotorType.kBrushless);
  public final CANSparkMax m_frontRightSpark = new CANSparkMax(Constants.frontRightSparkID, MotorType.kBrushless);
  public final CANSparkMax m_backLeftSpark = new CANSparkMax(Constants.backLeftSparkID, MotorType.kBrushless);
  public final CANSparkMax m_backRightSpark = new CANSparkMax(Constants.backRightSparkID, MotorType.kBrushless);

  // Locations of the wheels relative to the robot center.
  public final Translation2d m_frontLeftLocation = new Translation2d(Constants.wheelDisFromCenter, Constants.wheelDisFromCenter);
  public final Translation2d m_frontRightLocation = new Translation2d(Constants.wheelDisFromCenter, -Constants.wheelDisFromCenter);
  public final Translation2d m_backLeftLocation = new Translation2d(-Constants.wheelDisFromCenter, Constants.wheelDisFromCenter);
  public final Translation2d m_backRightLocation = new Translation2d(-Constants.wheelDisFromCenter, -Constants.wheelDisFromCenter);

  // Creating my kinematics object using the wheel locations.
  public final MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private AHRS gyro = new AHRS(SPI.Port.kMXP); 
  private Double gyroHold = null;

  //XboxController driveJoystick = new XboxController(0);
  public final XboxController driveJoystick = null;
  public final Joystick driveJoystickMain = new Joystick(0);
  public final Joystick driveJoystickSide = new Joystick(1);

  //set booleans
  public final boolean useGyroHold = true;
  public final boolean usingXboxController = driveJoystick != null;

  private MoveBy moveBy = new MoveBy(this, 5);
  private DriveTeleop drive = new DriveTeleop(this);
  private boolean shouldDrive = false;
  public final boolean tuningPID = false;
  
  public Drive() {
    init();
  }

  @Override
  public void periodic() {
    //init gyro
    if (gyro == null) {
      gyro = new AHRS(SPI.Port.kMXP);
      if (!gyro.isConnected()) gyro = null;
      // sHAKUANDO WAS HERE
      else gyro.calibrate();
    }
    else {
      SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
      SmartDashboard.putNumber("Gyro Rate", gyro.getRate());
    }
    
    //
    if (usingXboxController) {
      if (driveJoystick.getRawButton(9)) {
        gyro.reset();
        gyroHold = 0.0;
      }
    }
    else {
      //reset gyro
      if (driveJoystickMain.getRawButton(8)) {
        gyro.reset();
        gyroHold = 0.0;
      }
      //reset encoders
      if (driveJoystickMain.getRawButton(9)) setEncoderPos(0);
      
      //moveBy forward
      if (driveJoystickMain.getRawButtonPressed(3)) {
        moveBy.setMove(10);
        if (!moveBy.isScheduled()) moveBy.schedule();
        else moveBy.cancel();
      }
      //moveBy backwards
      if (driveJoystickMain.getRawButtonPressed(2)) {
        moveBy.setMove(-10);
        if (!moveBy.isScheduled()) moveBy.schedule();
        else moveBy.cancel();
      }
    }

    SmartDashboard.putBoolean("moveby", moveBy.isScheduled());

    //schedule drive accordingly
    if (shouldDrive) {
      if (!drive.isScheduled()) drive.schedule();
    }
    else {
      if (drive.isScheduled()) drive.cancel();
    }
  }

  //utility methods
  public void init() {
    m_frontLeftSpark.restoreFactoryDefaults(true);
    m_frontRightSpark.restoreFactoryDefaults(true);
    m_backLeftSpark.restoreFactoryDefaults(true);
    m_backRightSpark.restoreFactoryDefaults(true);

    m_frontLeftSpark.getEncoder();
    m_frontRightSpark.getEncoder();
    m_backLeftSpark.getEncoder();
    m_backRightSpark.getEncoder();

    setEncoderPos(0);

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

    setAllPIDControllers(Constants.defaultPID);
    setAllPIDControllers(Constants.velocityPID);

    m_frontLeftSpark.setInverted(false);
    m_frontRightSpark.setInverted(true);
    m_backLeftSpark.setInverted(false);
    m_backRightSpark.setInverted(true);
  }
  public void stop() {
    m_frontLeftSpark.set(0);
    m_frontRightSpark.set(0);
    m_backLeftSpark.set(0);
    m_backRightSpark.set(0);
  }

  //getters & setters
  public void setGyroHold(Double val) {
    this.gyroHold = val;
  }
  public Double getGyroHold() {
    return gyroHold;
  }
  public double getAngle() {
    return gyro.getAngle();
  }
  public double getRate() {
    return gyro.getRate();
  }

  public XboxController getDriveJoystick() {
    return driveJoystick;
  }
  public Joystick getDriveJoystickMain() {
    return driveJoystickMain;
  }
  public Joystick getDriveJoystickSide() {
    return driveJoystickSide;
  }

  public void setShouldDrive(boolean drive) {
    this.shouldDrive = drive;
  }
  public boolean getShouldDrive() {
    return shouldDrive;
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

  public void setFrontLeft(double val, ControlType type) {
    m_frontLeftSpark.getPIDController().setReference(val, type, Constants.defaultPID.kSlot);
  }
  public void setBackLeft(double val, ControlType type) {
    m_backLeftSpark.getPIDController().setReference(val, type, Constants.defaultPID.kSlot);
  }
  public void setFrontRight(double val, ControlType type) {
    m_frontRightSpark.getPIDController().setReference(val, type, Constants.defaultPID.kSlot);
  }
  public void setBackRight(double val, ControlType type) {
    m_backRightSpark.getPIDController().setReference(val, type, Constants.defaultPID.kSlot);
  }

  public RelativeEncoder getFrontLeftEncoder() {
    return m_frontLeftSpark.getEncoder();
  }
  public RelativeEncoder getFrontRightEncoder() {
    return m_frontRightSpark.getEncoder();
  }
  public RelativeEncoder getRearLeftEncoder() {
    return m_backLeftSpark.getEncoder();
  }
  public RelativeEncoder getRearRightEncoder() {
    return m_backRightSpark.getEncoder();
  }
  public void setEncoderPos(double position) {
    m_frontLeftSpark.getEncoder().setPosition(position);
    m_frontRightSpark.getEncoder().setPosition(position);
    m_backLeftSpark.getEncoder().setPosition(position);
    m_backRightSpark.getEncoder().setPosition(position);
  }
}
