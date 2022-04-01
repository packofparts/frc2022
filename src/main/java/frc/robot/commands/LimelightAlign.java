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

public class LimelightAlign extends CommandBase {
  /** Creates a new LidarMove. */
  DriveSubsystem drive;
  Limelight limelight;
  double originalYaw;

  Double PIDTurnDegrees;

  PIDController pid = new PIDController(Constants.limelightMovePID[0], Constants.limelightMovePID[1], Constants.limelightMovePID[2]);
  PIDController turnPID = new PIDController(Constants.limelightTurnPID[0], Constants.limelightTurnPID[1], Constants.limelightTurnPID[2]);
  
  public LimelightAlign(DriveSubsystem dt, Limelight limelight) {
    drive = dt;
    this.limelight = limelight;
    addRequirements(drive);
  }

  public LimelightAlign(DriveSubsystem dt, Limelight limelight, double PIDTurnDegrees) {
    drive = dt;
    this.limelight = limelight;
    this.PIDTurnDegrees = PIDTurnDegrees;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnPID.setTolerance(Constants.turnPIDTolerance);
    limelight.setPipeline(Pipeline.hub);

    drive.setShouldDrive(false);
    originalYaw = drive.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turn = 0;
    double move = 0;
    
    if (limelight.getDetected()) {
      turn = turnPID.calculate(limelight.getTX(), 0);
      move = pid.calculate(limelight.getTY(), 0);
    }
    else if (PIDTurnDegrees != null) {
      turn = -turnPID.calculate(drive.getAngle(), originalYaw + PIDTurnDegrees);
    }
    
    drive.drive(0, move, turn, false);
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
    return limelight.getDetected() && Math.abs(limelight.getTX()) < 0.25 && Math.abs(limelight.getTY()) < 0.25;
  }
}