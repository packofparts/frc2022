// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;

public class AutoAlign extends CommandBase {
  /** Creates a new AutoAlign. */
  DriveSubsystem driveBase;
  PIDController pid = new PIDController(Constants.LimelightKP, Constants.LimelightKI, Constants.LimelightKD);
  PIDController turnPID = new PIDController(Constants.turnPID[0], Constants.turnPID[1], Constants.turnPID[2]);

  //Creates network table
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public AutoAlign(DriveSubsystem dt) {
    driveBase = dt;
    addRequirements(driveBase);
  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
    return false;
  }
}
