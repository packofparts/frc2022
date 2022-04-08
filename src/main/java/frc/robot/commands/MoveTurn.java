// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.constants.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
public class MoveTurn extends CommandBase {
  DriveSubsystem drive;
  Limelight limelight;
  double initialPos;
  double move = 0;
  double turnTarget;

  PIDController movePID = new PIDController(Constants.movePID.kP, Constants.movePID.kI, Constants.movePID.kD);
  PIDController turnPID = new PIDController(Constants.turnPID[0], Constants.turnPID[1], Constants.turnPID[2]);

  public MoveTurn(DriveSubsystem drive, double move, double turnTarget) {
    this.drive = drive;
    this.move = move;
    this.turnTarget = turnTarget;
  }

  public MoveTurn(DriveSubsystem drive, Limelight limelight, double move, double turnTarget) {
    this.drive = drive;
    this.limelight = limelight;
    this.move = move;
    this.turnTarget = turnTarget;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPos = drive.getAveragePos();
    drive.setShouldDrive(false);  
    drive.stop();
    //initialTurn = drive.getAngle(); //amonggus
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turn = 0.0;
    if (limelight != null && limelight.getDetected()) turn = turnPID.calculate(limelight.getTX(), 0);
    else turn = -turnPID.calculate(drive.getAngle(), turnTarget);
    double speed = movePID.calculate(drive.getAveragePos(), initialPos+move);
    
    drive.drive(speed*Math.sin(drive.getAngle()*Math.PI/180), speed*Math.cos(drive.getAngle()*Math.PI/180), turn, true);
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
    return (Math.abs(drive.getAveragePos()-(initialPos+move)) < Constants.encoderDeadzone);
  }
}
