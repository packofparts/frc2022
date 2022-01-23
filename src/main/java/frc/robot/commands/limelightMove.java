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
import frc.robot.subsystems.Drive;
import edu.wpi.first.math.controller.PIDController;

public class limelightMove extends CommandBase {
  /** Creates a new LidarMove. */
  Drive driveBase = new Drive();
  PIDController pid = new PIDController(Constants.LimelightKP, Constants.LimelightKI, Constants.LimelightKD);
  //Creates network table
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  
  public limelightMove(Drive dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveBase);
    driveBase = dt;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableEntry currentHorizontal = table.getEntry("thor");
    NetworkTableEntry currentVertical = table.getEntry("tvert");
    //Gets actual value from spot on network table
    double thor = currentHorizontal.getDouble(0);
    double tvert = currentVertical.getDouble(0);
    // Pid- first value is current, second value is set point
    double speed = pid.calculate(thor*tvert, Constants.thor * Constants.tvert);
    driveBase.m_backLeftSpark.set(-speed);
    driveBase.m_frontLeftSpark.set(-speed);
    driveBase.m_backRightSpark.set(speed);
    driveBase.m_frontRightSpark.set(speed);
    // Gets spot on network table

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
