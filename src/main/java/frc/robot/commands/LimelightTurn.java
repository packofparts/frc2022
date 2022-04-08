// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.Pipeline;

public class LimelightTurn extends CommandBase {
  DriveSubsystem drive;
  Limelight limelight;
  PIDController pid = new PIDController(Constants.turnPID[0], Constants.turnPID[1], Constants.turnPID[2]);
  double PIDTurnDegrees;
  double originalYaw;

  public LimelightTurn(DriveSubsystem dt, Limelight limelight, double degrees) {
    drive = dt;
    this.limelight = limelight;
    addRequirements(drive);

    this.PIDTurnDegrees = degrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    originalYaw = drive.getAngle();
    drive.setShouldDrive(false);
    limelight.setPipeline(Pipeline.hub);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 0;
    if (limelight.getDetected()) speed = pid.calculate(limelight.getTX(), 0);
    else speed = -pid.calculate(drive.getAngle(), originalYaw + PIDTurnDegrees);

    drive.drive(0, 0, speed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    drive.setShouldDrive(true);
    limelight.setPipeline(Pipeline.drive);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(drive.getAngle()-(originalYaw+PIDTurnDegrees)) < Constants.gyroDeadzone || 
    (limelight.getDetected() && Math.abs(limelight.getTX()) < Constants.gyroDeadzone);
  }
}