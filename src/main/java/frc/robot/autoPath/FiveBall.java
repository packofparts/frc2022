// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

public class FiveBall extends CommandBase {
  boolean isFinished = false;
  int step = 0;

  DriveSubsystem drive;
  Tube tube;
  Shooter shooter;
  Limelight limelight;

  Command currentCommand;

  public FiveBall(DriveSubsystem drive, Tube tube, Shooter shooter, Limelight limelight) {
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (currentCommand == null || !currentCommand.isScheduled()) {
      //move forward 5.25 ft while intaking
      if (step == 0) {
        tube.setTubeMode(TubeMode.intake);
        currentCommand = new MoveBy(drive, 5.25);
      }
      //turn 180
      else if (step == 1) {
        tube.setTubeMode(TubeMode.off);
        currentCommand = new LimelightTurn(drive, 180, limelight);
      }
      //shoot
      else if (step == 2 && shooter.getShooterReady()) {
        tube.setTubeMode(TubeMode.feed);
        currentCommand = new TimerCommand(3);
        shooter.runShooter();
      }
      //turn 122 degrees (not final)
      else if (step == 3) {
        currentCommand = new TurnBy(drive, 122);
      }
      //go forward 10 ft (not final)
      else if (step == 4) {
        tube.setTubeMode(TubeMode.intake);
        currentCommand = new MoveBy(drive, 10);
      }
      //turn 105 degrees
      else if (step == 5) {
        tube.setTubeMode(TubeMode.off);
        currentCommand = new TurnBy(drive, 105);
      }
      //shoot
      else if (step == 6 && shooter.getShooterReady()) {
        tube.setTubeMode(TubeMode.feed);
        currentCommand = new TimerCommand(3);
        shooter.runShooter();
      }
      //turn 180 degrees
      else if (step == 7) {
        currentCommand = new TurnBy(drive, 180);
      }
      //go forward 12ft while intaking
      else if (step == 8) {
        tube.setTubeMode(TubeMode.intake);
        currentCommand = new MoveBy(drive, 12);
      }
      //pause for 3 seconds to let human player feed ball
      else if (step == 9) currentCommand = new TimerCommand(3);
      //turn 180
      else if (step == 10) {
        tube.setTubeMode(TubeMode.off);
        currentCommand = new TurnBy(drive, 180);
      }
      //go forward 12 ft again
      else if (step == 11) {
        currentCommand = new MoveBy(drive, 12);
      }
      //shoot
      else if (step == 12 && shooter.getShooterReady()) {
        tube.setTubeMode(TubeMode.feed);
        currentCommand = new TimerCommand(3);
        shooter.runShooter();
      }
      //spin to celebrate
      else if (step == 13) currentCommand = new TurnBy(drive, 720);
      //stop robot
      else if (step == 14) isFinished = true;
 
      currentCommand.schedule();
      step++;
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
