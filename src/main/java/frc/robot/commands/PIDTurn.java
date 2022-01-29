// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
public class PIDTurn extends CommandBase {
  DriveSubsystem driveBase;
  PIDController pid = new PIDController(0, 0, 0);
  public PIDTurn(DriveSubsystem dt, double Degrees) {
    driveBase = dt;
    addRequirements(driveBase);
    Constants.PIDTurnDegrees = Degrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.originalYaw = driveBase.getAngle();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pid.calculate(driveBase.getAngle(), Constants.originalYaw + Constants.PIDTurnDegrees);
    driveBase.setBackLeftVelocity(-speed);
    driveBase.setFrontLeftVelocity(-speed);
    driveBase.setBackRightVelocity(speed);
    driveBase.setFrontRightVelocity(speed);
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
