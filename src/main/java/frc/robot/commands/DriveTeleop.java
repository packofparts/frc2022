// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.stream.DoubleStream;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class DriveTeleop extends CommandBase {
  Drive drive = new Drive();
  
  PIDController turnPID = new PIDController(Constants.turnPID[0], Constants.turnPID[1], Constants.turnPID[2]);
  PIDController ratePID = new PIDController(Constants.ratePID[0], Constants.ratePID[1], Constants.ratePID[2]);

  public DriveTeleop(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!drive.tuningPID) drive();
    else tunePID();
  }

  private void drive() {
    double xSpeed = 0;
    double ySpeed = 0;
    double rotation = 0;

    //controller inputs
    if (drive.usingXboxController) {
      xSpeed = drive.getDriveJoystick().getRightX();
      ySpeed = -drive.getDriveJoystick().getRightY();
      rotation = -drive.getDriveJoystick().getLeftX();
    }
    else {
      xSpeed = -drive.getDriveJoystickMain().getX();
      ySpeed = -drive.getDriveJoystickMain().getY();
      rotation = -drive.getDriveJoystickSide().getX();
    }

    //scaling
    // double turnFactor = (1-fastestTurn) * Math.pow(1-Math.pow(fastestTurn, 8/3), 6) + minTurn;
    // xSpeed = ((1-minPower) * Math.abs(Math.pow(xSpeed, 8/3)) + minPower) * getSign(xSpeed);
    // ySpeed = ((1-minPower) * Math.abs(Math.pow(ySpeed, 8/3)) + minPower) * getSign(ySpeed);
    // rotation = ((1-minTurn) * Math.abs(Math.pow(turn, 8/3)) + minTurn) * getSign(turn) * turnFactor;

    drive(xSpeed, ySpeed, rotation, true);
  }

  private void drive(double xSpeed, double ySpeed, double rotation, boolean deadZone) {
    //deadzone
    if (deadZone) {
      if (Math.abs(xSpeed) < Constants.joystickDeadzone) {
        xSpeed = 0;
      }
      if (Math.abs(ySpeed) < Constants.joystickDeadzone) {
        ySpeed = 0;
      }
      if (Math.abs(rotation) < Constants.joystickDeadzone) {
        rotation = 0;
      }
    }

    //normalize
    if (xSpeed != 0) {
      xSpeed = Constants.maxSpeed * xSpeed;
    }
    if (ySpeed != 0) {
      ySpeed = Constants.maxSpeed * ySpeed;
    }
    if (rotation != 0) {
      rotation = ratePID.calculate(drive.getRate(), rotation*Constants.maxRate) * Constants.maxTurnOutput;
      if (drive.getGyroHold() != null) drive.setGyroHold(null);
    }
    //GYRO HOLD
    else {
      if (drive.useGyroHold) {
        if (drive.getGyroHold() == null && Math.abs(drive.getRate()) < Constants.gyroDeadzone) {
          drive.setGyroHold(drive.getAngle());
        }

        if (drive.getGyroHold() != null) {
          rotation = ratePID.calculate((-turnPID.calculate(drive.getAngle(), drive.getGyroHold()))*Constants.maxRate)*Constants.maxTurnOutput;
        }
      }
    }

    double gyroHoldTemp = 0.0;
    if (drive.getGyroHold() != null)  gyroHoldTemp = drive.getGyroHold();
    SmartDashboard.putNumber("gyroHold", gyroHoldTemp);
    SmartDashboard.putNumber("rotation", rotation);

    // Convert to wheel speeds
    ChassisSpeeds speeds;
    if (drive.usingXboxController) {
      if (drive.getDriveJoystick().getRawButton(10)) speeds = new ChassisSpeeds(ySpeed, xSpeed, rotation);
      else speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rotation, Rotation2d.fromDegrees(-drive.getAngle()));
    }
    else {
      if (drive.getDriveJoystickMain().getRawButton(1)) speeds = new ChassisSpeeds(ySpeed, xSpeed, rotation);
      else speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rotation, Rotation2d.fromDegrees(-drive.getAngle()));
    }

    MecanumDriveWheelSpeeds wheelSpeeds = drive.m_kinematics.toWheelSpeeds(speeds);
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
    drive.setFrontLeft(frontLeft, ControlType.kVelocity);
    drive.setFrontRight(frontRight, ControlType.kVelocity);
    drive.setBackLeft(backLeft, ControlType.kVelocity);
    drive.setBackRight(backRight, ControlType.kVelocity);

    SmartDashboard.putNumber("frontLeftSet", frontLeft);
    SmartDashboard.putNumber("frontRightSet", frontRight);
    SmartDashboard.putNumber("rearLeftSet", backLeft);
    SmartDashboard.putNumber("rearRightSet", backRight);

    SmartDashboard.putNumber("frontLeftPos", drive.getFrontLeftEncoder().getPosition());
    SmartDashboard.putNumber("frontRightPos", drive.getFrontRightEncoder().getPosition());
    SmartDashboard.putNumber("rearLeftPos", drive.getRearLeftEncoder().getPosition());
    SmartDashboard.putNumber("rearRightPos", drive.getRearRightEncoder().getPosition());
    
    SmartDashboard.putNumber("frontLeftVel", drive.getFrontLeftEncoder().getVelocity());
    SmartDashboard.putNumber("frontRightVel", drive.getFrontRightEncoder().getVelocity());
    SmartDashboard.putNumber("rearLeftVel", drive.getRearLeftEncoder().getVelocity());
    SmartDashboard.putNumber("rearRightVel", drive.getRearRightEncoder().getVelocity());
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

  public void tunePID() {
    SmartDashboard.putNumber("P: ", ratePID.getP());
    SmartDashboard.putNumber("I: ", ratePID.getI());
    SmartDashboard.putNumber("D: ", ratePID.getD());

    if (drive.getDriveJoystickSide().getRawButtonReleased(6)) ratePID.setP(ratePID.getP() + 0.05);
    if (drive.getDriveJoystickSide().getRawButtonReleased(7)) ratePID.setP(ratePID.getP() - 0.05);
    if (drive.getDriveJoystickSide().getRawButtonReleased(8)) ratePID.setI(ratePID.getI() + 0.0005);
    if (drive.getDriveJoystickSide().getRawButtonReleased(9)) ratePID.setI(ratePID.getI() - 0.0005);
    if (drive.getDriveJoystickSide().getRawButtonReleased(11)) ratePID.setD(ratePID.getD() + 0.0005);
    if (drive.getDriveJoystickSide().getRawButtonReleased(10)) ratePID.setD(ratePID.getD() - 0.0005);

    if (drive.getDriveJoystickSide().getRawButton(4)) {
      drive(0, 0, -ratePID.calculate(drive.getRate(), 0.5), false);
      System.out.println("slow");
    }
    else if (drive.getDriveJoystickSide().getRawButton(5)) {
      drive(0, 0, -ratePID.calculate(drive.getRate(), 1), false);
      System.out.println("fast");
    }
    else {
      drive(0, 0, 0, false);
    }

    // if (drive.getDriveJoystickSide().getRawButton(4)) {
    //   drive(0, 0, -turnPID.calculate(drive.getAngle(), -90), false);
    // }
    // else if (drive.getDriveJoystickSide().getRawButton(5)) {
    //   drive(0, 0, -turnPID.calculate(drive.getAngle(), 90), false);
    // }
    // else if (drive.getDriveJoystickSide().getRawButton(2)) {
    //   drive(0, 0, -turnPID.calculate(drive.getAngle(), 10), false);
    // }
    // else if (drive.getDriveJoystickSide().getRawButton(3)) {
    //   drive(0, 0, -turnPID.calculate(drive.getAngle(), 0), false);
    // }
    // else {
    //   drive(0, 0, 0, false);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
