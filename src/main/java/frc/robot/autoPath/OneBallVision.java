// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoPath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.LimelightAlign;
import frc.robot.commands.TimerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tube;
import frc.robot.subsystems.Shooter.ShooterMode;
import frc.robot.subsystems.Tube.TubeMode;

public class OneBallVision extends CommandBase {
  boolean isFinished = false;
  int step = 0;

  DriveSubsystem drive;
  Tube tube;
  Shooter shooter;
  Limelight limelight;

  Command currentCommand;

  Timer feedTimer;

  public OneBallVision(DriveSubsystem drive, Tube tube, Shooter shooter, Limelight limelight) {
    this.drive = drive;
    this.tube = tube;
    this.shooter = shooter;
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setShouldDrive(false);
    tube.setUseJoysticks(false);
    //extend pneumatics
    tube.setPneumatics(true);
    //set intake mode
    tube.setTubeMode(TubeMode.intake);
    //set shooter mode
    shooter.setShooterMode(ShooterMode.normal);
    
    step = 0;
    currentCommand = null;

    feedTimer = new Timer();
    feedTimer.reset();
  }

  //Called every time the scheduler runs while the command is scheduled.
  //TODO check step 1 limelight align, step 2 running & feed
  @Override
  public void execute() {
    boolean feed = false;

    if (currentCommand == null || !currentCommand.isScheduled()) {
      boolean next = true;

      //align to hub
      if (step == 0) currentCommand = new LimelightAlign(drive, limelight);
      //feedShooter for 10 seconds
      else if (step == 1 && shooter.getShooterReady()) {
        feed = true;
        tube.setTubeMode(TubeMode.feed);
        currentCommand = new TimerCommand(10);
      }
      else if (step == 1 & !shooter.getShooterReady()) next = false;
      //stop robot
      else if (step == 2) isFinished = true;

      //manage step increments
      if (next) {
        currentCommand.schedule();
        step++;
      }
    }

    //manage feeding
    if (feed) {
      if (shooter.getShooterReady()) {
        feedTimer.start();
      }
      else if (shooter.getShooterReady() && feedTimer.get() > 0.5)  {
        tube.setTubeMode(TubeMode.feed);
        feedTimer.stop();
      }
      else {
        tube.setTubeMode(TubeMode.off);
        feedTimer.stop();
        feedTimer.reset();
      }
    }
    else {
      feedTimer.stop();
      feedTimer.reset();
    }

    //run shooter and tube
    shooter.runShooter();
    tube.runTube();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    tube.stopTube();
    shooter.stopShooter();

    drive.setShouldDrive(true);
    tube.setUseJoysticks(true);
    if (currentCommand != null) currentCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}