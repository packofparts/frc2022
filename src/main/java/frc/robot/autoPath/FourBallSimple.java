// Copyright (c) FIRST and other WPILib contributors. amonugds
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoPath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.LimelightAlign;
import frc.robot.commands.LimelightTurn;
import frc.robot.commands.MoveBy;
import frc.robot.commands.TurnBy;
import frc.robot.commands.TurnTo;
import frc.robot.commands.TimerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tube;
import frc.robot.subsystems.Shooter.ShooterMode;
import frc.robot.subsystems.Tube.TubeMode;

public class FourBallSimple extends CommandBase {
  boolean isFinished = false;
  int step = 0;

  DriveSubsystem drive;
  Tube tube;
  Shooter shooter;
  Limelight limelight;

  Command currentCommand;

  double initalAngle;

  Timer feedTimer;

  public FourBallSimple(DriveSubsystem drive, Tube tube, Shooter shooter, Limelight limelight) {
    this.drive = drive;
    this.tube = tube;
    this.shooter = shooter;
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tube.setUseJoysticks(false);
    //extend pneumatics
    tube.setPneumatics(true);
    //set intake mode
    tube.setTubeMode(TubeMode.intake);
    //set shooter mode
    shooter.setShooterMode(ShooterMode.normal);

    step = 0;
    currentCommand = null;
    initalAngle = drive.getAngle();
    
    feedTimer = new Timer();
    feedTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  //TODO: check step 1 dis, step 2 running (feed & next), step 7 align, step 8 running
  @Override
  public void execute() {
    boolean feed = false;

    if (currentCommand == null || !currentCommand.isScheduled()) {
      boolean next = true;

      //move forward 5.25 ft while intaking
      if (step == 0) {
        tube.setTubeMode(TubeMode.intake);
        currentCommand = new MoveBy(drive, 6.5); //5.25
      }
      //turn 180
      else if (step == 1) {
        tube.setTubeMode(TubeMode.off);
        currentCommand = new LimelightTurn(drive, 180, limelight);
      }
      //shoot only when ready
      else if (step == 2 && shooter.getShooterReady()) {
        feed = true;
        tube.setTubeMode(TubeMode.feed);
        currentCommand = new TimerCommand(3);
      }
      else if (step == 2 && !shooter.getShooterReady()) next = false;
      //turn back onto course
      else if (step == 3) {
        tube.setTubeMode(TubeMode.off);
        shooter.setShooterMode(ShooterMode.off);
        currentCommand = new TurnTo(drive, initalAngle);
      }
      //intake while driving forward 12 ft
      else if (step == 4) { 
        tube.setTubeMode(TubeMode.intake);
        currentCommand = new MoveBy(drive, 12.35); //13.6
      }
      // wait for human player to drop ball
      else if (step == 5) currentCommand = new TimerCommand(3);
      //move backwards 12ft
      else if (step == 6) {
        tube.setTubeMode(TubeMode.off);
        shooter.setShooterMode(ShooterMode.normal);
        currentCommand = new MoveBy(drive, -12.35);
      }
      //turn to face the hub and align
      else if (step == 7) currentCommand = new LimelightAlign(drive, limelight, 180);
      //feedShooter for 3 seconds
      else if (step == 8 && shooter.getShooterReady()) {
        feed = true;
        tube.setTubeMode(TubeMode.feed);
        currentCommand = new TimerCommand(5);
      }
      else if (step == 8 && !shooter.getShooterReady()) next = false;
      //end execute
      else if (step == 9) isFinished = true;

      if (next) {
        currentCommand.schedule();
        step++;
      }
    }

    //manage feeding
    if (feed) {
      if (shooter.getShooterReady() && feedTimer.get() > 0.5)  {
        tube.setTubeMode(TubeMode.feed);
        feedTimer.stop();
        feedTimer.reset();
      }
      else {
        tube.setTubeMode(TubeMode.off);
        feedTimer.reset();
        feedTimer.start();
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

    tube.setUseJoysticks(true);
    if (currentCommand != null) currentCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}