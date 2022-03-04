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
  /** Creates a new LidarMove. */
  DriveSubsystem driveBase;
  Limelight limelight;
  Pipeline pipeline;
  double turnBy;
  double originalYaw;

  PIDController pid = new PIDController(Constants.limelightPID[0], Constants.limelightPID[1], Constants.limelightPID[2]);
  PIDController turnPID = new PIDController(Constants.limelightTurnPID[0], Constants.limelightTurnPID[1], Constants.limelightTurnPID[2]);
  
  public LimelightTurn(DriveSubsystem dt, Limelight limelight, Pipeline pipeline, double turnBy) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveBase = dt;
    this.limelight = limelight;
    this.pipeline = pipeline;
    this.turnBy = turnBy;
    addRequirements(driveBase);
  }

  public void setPipeline(Pipeline pipeline) {
    this.pipeline = pipeline;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnPID.setTolerance(Constants.turnPIDTolerance);
    limelight.setPipeline(pipeline);

    originalYaw = driveBase.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turn = 0;
    if (limelight.getDetected()) turn = -turnPID.calculate(driveBase.getAngle(), driveBase.getAngle()+limelight.getTX());
    else turn = pid.calculate(driveBase.getAngle(), originalYaw + turnBy);
    
    driveBase.drive(0, 0, turn, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.stop();
    limelight.setPipeline(Pipeline.drive);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (limelight.getDetected()) return limelight.getTX() < Constants.gyroDeadzone;
    else return Math.abs(driveBase.getAngle()-(originalYaw+turnBy)) < Constants.gyroDeadzone;
  }
}
