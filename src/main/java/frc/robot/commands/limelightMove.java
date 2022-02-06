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

public class limelightMove extends CommandBase {
  /** Creates a new LidarMove. */
  DriveSubsystem driveBase;
  Limelight limelight;
  Pipeline pipeline;
  double area;

  PIDController pid = new PIDController(Constants.limelightPID[0], Constants.limelightPID[1], Constants.limelightPID[2]);
  PIDController turnPID = new PIDController(Constants.limelightTurnPID[0], Constants.limelightTurnPID[1], Constants.limelightTurnPID[2]);
  
  public limelightMove(DriveSubsystem dt, Limelight limelight, Pipeline pipeline, double area) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveBase = dt;
    this.limelight = limelight;
    this.pipeline = pipeline;
    this.area = area;
    addRequirements(driveBase);
    addRequirements(limelight);
  }

  public void setPipeline(Pipeline pipeline) {
    this.pipeline = pipeline;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnPID.setTolerance(Constants.turnPIDTolerance);
    limelight.setPipeline(pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(pipeline);
    if (limelight.getDetected()) {
      double speed = pid.calculate(limelight.getHorizontal()*limelight.getVertical(), area);
      if (speed > 0.3) speed = 0.3;
      else if (speed < -0.3) speed = -0.3;

      double turn = -turnPID.calculate(driveBase.getAngle(), driveBase.getAngle()+limelight.getTX());

      driveBase.drive(0, speed, turn, false);
    }
    else {
      driveBase.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.setPipeline(Pipeline.drive);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
