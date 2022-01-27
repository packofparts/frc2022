// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.Constants;
public class PIDTurn extends CommandBase {
  
  Drive driveBase = new Drive();
  PIDController pid = new PIDController(0, 0, 0);
  public PIDTurn(Drive dt, double Degrees) {
    addRequirements(driveBase);
    driveBase = dt;
    Constants.PIDTurnDegrees = Degrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.originalYaw = driveBase.getAngle();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pid.calculate(driveBase.getAngle(), Constants.originalYaw + Constants.PIDTurnDegrees);
    driveBase.setBackLeft(-speed, ControlType.kVelocity);
    driveBase.setFrontLeft(-speed, ControlType.kVelocity);
    driveBase.setBackRight(speed, ControlType.kVelocity);
    driveBase.setFrontRight(speed, ControlType.kVelocity);
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
