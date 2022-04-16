// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoPath;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.LimelightTurn;
import frc.robot.commands.MoveBy;
import frc.robot.commands.TurnBy;
import frc.robot.commands.TimerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tube;
import frc.robot.subsystems.Shooter.ShooterMode;
import frc.robot.subsystems.Tube.TubeMode;

public class TrajectoryGeneration extends CommandBase {
  boolean isFinished = false;
  String trajectoryJSON = "Paths/5ball+defence.wpilib.json";
  Trajectory trajectory = new Trajectory();
  DriveSubsystem drive;
  // Called when the command is initially scheduled.
  public TrajectoryGeneration(DriveSubsystem drive){
    this.drive = drive;
  }
  @Override

  public void initialize() {
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
   }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      RamseteCommand command = new RamseteCommand(
        trajectory,
        drive::getpose,
        new RamseteController(2, .7),
        drive.getFeedForward(),
        drive.getKinematics(),
        drive::getSpeeds,
        new PIDController(9, 0, 0),
        new PIDController(9, 0, 0),
        drive::setOutputVolts,
        drive
    );
      
      
    return command.andThen(() -> drive.setOutputVolts(0, 0));
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
