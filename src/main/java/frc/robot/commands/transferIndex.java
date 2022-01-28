// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Index;
import edu.wpi.first.wpilibj.Timer;
public class transferIndex extends CommandBase {
  /** Creates a new transferIndex. */

  Index index;
  Timer timer;
  private boolean finish = false;
  public transferIndex(double indexSpeed, Index in) {
    index = in;
    timer = new Timer();
    addRequirements(index);
    Constants.indexSpeed = indexSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    while (timer.get() < Constants.spinTime) {
      index.indexMotor.set(ControlMode.Velocity, Constants.indexSpeed);
    }
    System.out.println("index complete");
    finish = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
