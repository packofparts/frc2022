// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class MoveBy extends CommandBase {
  Drive drive;
  double[] currentPos = new double[4];
  double move = 0;

  public MoveBy(Drive drive, double move) {
    this.drive = drive;
    this.move = move;
  }

  public void setMove(double move) {
    this.move = move;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPos[0] = drive.getFrontLeftEncoder().getPosition();
    currentPos[1] = drive.getFrontRightEncoder().getPosition();
    currentPos[2] = drive.getRearLeftEncoder().getPosition();
    currentPos[3] = drive.getRearRightEncoder().getPosition();
    drive.setShouldDrive(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setFrontLeft(currentPos[0]+move, ControlType.kPosition);
    drive.setFrontRight(currentPos[1]+move, ControlType.kPosition);
    drive.setBackLeft(currentPos[2]+move, ControlType.kPosition);
    drive.setBackRight(currentPos[3]+move, ControlType.kPosition);
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
    if (Math.abs((currentPos[0]+move)-drive.getFrontLeftEncoder().getPosition()) < Constants.encoderDeadzone &&
    Math.abs((currentPos[1]+move)-drive.getFrontRightEncoder().getPosition()) < Constants.encoderDeadzone &&
    Math.abs((currentPos[2]+move)-drive.getRearLeftEncoder().getPosition()) < Constants.encoderDeadzone &&
    Math.abs((currentPos[3]+move)-drive.getRearRightEncoder().getPosition()) < Constants.encoderDeadzone) {
      return true;
    }

    return false;
  }
}
