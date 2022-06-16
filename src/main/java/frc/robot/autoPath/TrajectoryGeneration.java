// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoPath;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.LimelightTurn;
import frc.robot.commands.MoveBy;
import frc.robot.commands.TurnBy;
import frc.robot.commands.TimerCommand;
import frc.robot.subsystems.DriveSubsystem;

public class TrajectoryGeneration extends CommandBase {
  boolean isFinished = false;
  String trajectoryJSON = "paths/5ball+defence.wpilib.json";
  Trajectory trajectory = new Trajectory();
  DriveSubsystem drive;
  Trajectory.State goal;
  Timer sus = new Timer();
  // -5 is a stopper number
  int temp = 0;
  HolonomicDriveController controller = new HolonomicDriveController(
    new PIDController(0, 0, 0), new PIDController(0, 0, 0),
    new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14)));
    // Tune TrapezoidProfile consstraints to turning speed and acceleration

  // Called when the command is initially scheduled.
  public TrajectoryGeneration(DriveSubsystem drive){
    this.drive = drive;
  }
  @Override

  public void initialize() {
    sus.start();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
   }
   Trajectory.State goal = trajectory.sample(0);
  } 


  @Override
  public void execute() {
    if (sus.get()< 8.75){
      Trajectory.State goal = trajectory.sample(sus.get());
      ChassisSpeeds adjustedSpeeds = controller.calculate(
        drive.getpose(), goal, Rotation2d.fromDegrees(70.0));
      double vx = adjustedSpeeds.vxMetersPerSecond;
      double vy = adjustedSpeeds.vyMetersPerSecond;
      double rotation = adjustedSpeeds.omegaRadiansPerSecond;
      drive.drive(vx,vy,rotation,false);
      System.out.println(" goal "+ goal);
    }else{
      end(false);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sus.stop();
    sus.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
