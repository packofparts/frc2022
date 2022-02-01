// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class PIDTurn extends CommandBase {
  DriveSubsystem driveBase;
  PIDController pid = new PIDController(0, 0, 0);
  double PIDTurnDegrees;
  double originalYaw = 0;

  public PIDTurn(DriveSubsystem dt, double degrees) {
    driveBase = dt;
    addRequirements(driveBase);
    this.PIDTurnDegrees = degrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    originalYaw = driveBase.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pid.calculate(driveBase.getAngle(), originalYaw + PIDTurnDegrees);
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
