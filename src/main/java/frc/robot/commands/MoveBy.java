// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.constants.Constants;
import frc.robot.subsystems.DriveSubsystem;
public class MoveBy extends CommandBase {
  /** Creates a new MoveBy. */
  DriveSubsystem drive;
  double[] currentPos = new double[4];
  double move = 0;

  public MoveBy(DriveSubsystem drive, double move) {
    this.drive = drive;
    this.move = move;
  }

  public void setMove(double move) {
    this.move = move;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPos[0] = drive.getFrontLeftPos();
    currentPos[1] = drive.getFrontRightPos();
    currentPos[2] = drive.getRearLeftPos();
    currentPos[3] = drive.getRearRightPos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setFrontLeftPosition(currentPos[0]+move);
    drive.setFrontRightPosition(currentPos[1]+move);
    drive.setBackLeftPosition(currentPos[2]+move);
    drive.setBackRightPosition(currentPos[3]+move);
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
    if (Math.abs((currentPos[0]+move)-drive.getFrontLeftPos()) < Constants.encoderDeadzone &&
    Math.abs((currentPos[1]+move)-drive.getFrontRightPos()) < Constants.encoderDeadzone &&
    Math.abs((currentPos[2]+move)-drive.getRearLeftPos()) < Constants.encoderDeadzone &&
    Math.abs((currentPos[3]+move)-drive.getRearRightPos()) < Constants.encoderDeadzone) {
      return true;
    }

    return false;
  }
}
