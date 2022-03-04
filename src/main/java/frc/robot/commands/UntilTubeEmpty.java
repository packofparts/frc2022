// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tube;

public class UntilTubeEmpty extends CommandBase {
  Tube tube;
  boolean empty;
  Timer timer;
  double time;

  public UntilTubeEmpty(Tube tube, double time) {
    this.tube = tube;
    this.time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.reset();

    empty = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!tube.getIntake() && !tube.getIndexFront() && !tube.getIndexBack()) {
      timer.start();
      if (timer.get() <= time) empty = true;
    }
    else if (timer.get() != 0) timer.reset();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return empty;
  }
}
