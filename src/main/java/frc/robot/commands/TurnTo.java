// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnTo extends CommandBase {
  DriveSubsystem drive;
  PIDController pid = new PIDController(Constants.turnPID[0], Constants.turnPID[1], Constants.turnPID[2]);
  double PIDTurnDegrees;

  public TurnTo(DriveSubsystem dt, double degrees) {
    drive = dt;
    addRequirements(drive);

    this.PIDTurnDegrees = degrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setShouldDrive(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -pid.calculate(drive.getAngle(), PIDTurnDegrees);
    drive.drive(0, 0, speed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    drive.setShouldDrive(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(drive.getAngle()-PIDTurnDegrees) < Constants.gyroDeadzone;
  }
}