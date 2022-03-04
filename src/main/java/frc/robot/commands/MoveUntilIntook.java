// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.constants.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Tube;
import frc.robot.subsystems.Limelight.Pipeline;
public class MoveUntilIntook extends CommandBase {
  DriveSubsystem drive;
  Tube tube;
  Pipeline pipeline;
  double initialPos;
  double move = 0;

  boolean intook;
  Timer timer;
  double time;

  PIDController movePID = new PIDController(Constants.movePID.kP, Constants.movePID.kI, Constants.movePID.kD);

  public MoveUntilIntook(DriveSubsystem drive, Tube tube, double move, Pipeline pipeline, double time) {
    this.drive = drive;
    this.tube = tube;
    this.move = move;
    this.pipeline = pipeline;
    this.time = time;
  }

  public void setMove(double move) {
    this.move = move;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPos = drive.getAveragePos();
    intook = false;

    timer = new Timer();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = movePID.calculate(drive.getAveragePos(), initialPos+move);
    drive.drive(0, speed, 0, false);

    if (tube.getIntake(pipeline)) {
      if (timer.get() == 0) timer.start();
      else if (timer.get() >= time) intook = true;
    }
    else if (timer.get() != 0) timer.reset();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(drive.getAveragePos()-initialPos) < Constants.encoderDeadzone) || intook;
  }
}
