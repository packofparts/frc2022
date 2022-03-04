// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.LimelightTurn;
import frc.robot.commands.MoveBy;
import frc.robot.commands.TurnBy;
import frc.robot.commands.TurnTo;
import frc.robot.commands.TimerCommand;
import frc.robot.subsystems.DriveSubsystem;
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

  Command currentCommand;

  public FourBallSimple(DriveSubsystem drive, Tube tube, Shooter shooter) {
    this.drive = drive;
    this.tube = tube;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //extend pneumatics
    tube.setPneumatics(true);
    //set intake mode
    tube.setTubeMode(TubeMode.intake);
    //set shooter mode
    shooter.setShooterMode(ShooterMode.auto);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!currentCommand.isScheduled()) {
      //move forward 4 ft
      if (step == 0) currentCommand = new MoveBy(drive, 4);
      //rotate 180
      else if (step == 1) currentCommand = new TurnBy(drive, 180);
      //feedShooter for 3 seconds
      else if (step == 2) {
        currentCommand = new TimerCommand(3);
        tube.setTubeMode(TubeMode.feed);
      }
      //turns robot 180 degrees && stop intaking
      else if (step == 3) {
        tube.setTubeMode(TubeMode.off);
        currentCommand = new TurnBy(drive, 180);
      }
      //intake while driving forward 10 ft
      else if (step == 4) {
        tube.setTubeMode(TubeMode.intake);
        currentCommand = new MoveBy(drive, 10.275);
      }
      //pause for 3 seconds to let human player feed ball
      else if (step == 5) currentCommand = new TimerCommand(3);
      //turn robot to zero degrees
      else if (step == 6) currentCommand = new TurnTo(drive, 0);
      //move forward 12ft
      else if (step == 7) currentCommand = new MoveBy(drive, 12);
      //turn to face the hub
      else if (step == 8) currentCommand = new TurnTo(drive, -45);
      //feedShooter for 3 seconds
      else if (step == 9) {
        currentCommand = new TimerCommand(3);
        tube.setTubeMode(TubeMode.feed);
      }
      //end execute
      else if (step == 10) isFinished = true;

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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}