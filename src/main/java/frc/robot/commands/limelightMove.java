// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class limelightMove extends CommandBase {
  /** Creates a new LidarMove. */
  DriveSubsystem driveBase;
  PIDController pid = new PIDController(Constants.limelightPID[0], Constants.limelightPID[1], Constants.limelightPID[2]);
  PIDController turnPID = new PIDController(Constants.limelightTurnPID[0], Constants.limelightTurnPID[1], Constants.limelightTurnPID[2]);

  //Creates network table
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  
  public limelightMove(DriveSubsystem dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveBase = dt;
    addRequirements(driveBase);
    System.out.println("HERE");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnPID.setTolerance(Constants.turnPIDTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (table.getEntry("tv").getDouble(0) == 1) {
      NetworkTableEntry currentHorizontal = table.getEntry("thor");
      NetworkTableEntry currentVertical = table.getEntry("tvert");
      // double currentArea = table.getEntry("targetArea").getDouble(0);
      //Gets actual value from spot on network table
      double thor = currentHorizontal.getDouble(0);
      double tvert = currentVertical.getDouble(0);
      
      // Pid- first value is current, second value is set point
      double speed = pid.calculate(thor*tvert, Constants.thor * Constants.tvert);
      if (speed > 0.3) speed = 0.3;
      else if (speed < -0.3) speed = -0.3;

      double tx = table.getEntry("tx").getDouble(0);
      double turn = -turnPID.calculate(driveBase.getAngle(), driveBase.getAngle()+tx);
      System.out.println(turn);

      driveBase.drive(0, speed, turn, false);
    }
    else {
      driveBase.stop();
    }
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
