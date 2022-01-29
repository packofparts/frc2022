// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;

public class AutoAlign extends CommandBase {
  /** Creates a new Lidar. */
  double currentAngle;
  Limelight limelight;
  boolean isFinished;

  public AutoAlign() {
    // Use addRequirements() here to declare subsystem dependencies.
    limelight = new Limelight();
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //find current angle
    currentAngle = limelight.tx;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(currentAngle != 0){
      // turn currentAngle using PIDTurn from auto branch
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(currentAngle == 0) isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
