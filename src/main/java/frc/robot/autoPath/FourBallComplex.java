// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.LimelightTurn;
import frc.robot.commands.MoveBy;
import frc.robot.commands.MoveUntilIntook;
import frc.robot.commands.TurnTo;
import frc.robot.commands.UntilTubeEmpty;
import frc.robot.commands.TimerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tube;
import frc.robot.subsystems.Limelight.Pipeline;
import frc.robot.subsystems.Shooter.ShooterMode;
import frc.robot.subsystems.Tube.TubeMode;

public class FourBallComplex extends CommandBase {
  boolean isFinished = false;
  int step = 0;

  double initPathAngle;

  Robot robot;
  DriveSubsystem drive;
  Tube tube;
  Shooter shooter;
  Limelight limelight;

  Command currentCommand;

  public FourBallComplex(Robot robot, DriveSubsystem drive, Tube tube, Shooter shooter, Limelight limelight) {
    this.robot = robot;
    this.drive = drive;
    this.tube = tube;
    this.shooter = shooter;
    this.limelight = limelight;
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

    initPathAngle = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!currentCommand.isScheduled()) {
      //move forward 4 ft or until ball is in intake for 0.5 seconds
      if (step == 0) currentCommand = new MoveUntilIntook(drive, tube, 4, robot.getBallColor(), 0.5);
      //rotate 180 or until centered
      else if (step == 1) {
        initPathAngle = drive.getAngle();
        currentCommand = new LimelightTurn(drive, limelight, Pipeline.hub, 180);
      }
      //feedShooter until all balls are fed
      else if (step == 2) {
        currentCommand = new UntilTubeEmpty(tube, 0.5);
        tube.setTubeMode(TubeMode.feed);
      }
      //turns robot back onto original path && stop intaking
      else if (step == 3) {
        tube.setTubeMode(TubeMode.off);
        currentCommand = new TurnTo(drive, initPathAngle);
      }
      //intake while driving forward 10 ft or until ball intook
      else if (step == 4) {
        tube.setTubeMode(TubeMode.intake);
        currentCommand = new MoveUntilIntook(drive, tube, 10.275, robot.getBallColor(), 0.5);
      }
      //pause for 3 seconds to let human player feed ball
      else if (step == 5) currentCommand = new TimerCommand(3);
      //turn robot to zero degrees
      else if (step == 6) currentCommand = new TurnTo(drive, 0);
      //move forward 12ft
      else if (step == 7) currentCommand = new MoveBy(drive, 12);
      //turn to face the hub
      else if (step == 8) currentCommand = new LimelightTurn(drive, limelight, Pipeline.hub, -45);
      //feedShooter until all balls are fed
      else if (step == 9) {
        currentCommand = new UntilTubeEmpty(tube, 0.5);
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