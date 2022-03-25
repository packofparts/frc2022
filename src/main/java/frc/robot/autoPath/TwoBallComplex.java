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

public class TwoBallComplex extends CommandBase {
  boolean isFinished = false;
  int step = 0;

  DriveSubsystem drive;
  Tube tube;
  Shooter shooter;
  Limelight limelight;

  Command currentCommand;

  public TwoBallComplex(DriveSubsystem drive, Tube tube, Shooter shooter, Limelight limelight) {
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
    shooter.setShooterMode(ShooterMode.auto);
    
    step = 0;
    currentCommand = null;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (currentCommand == null || !currentCommand.isScheduled()) {
      //move forward 4 ft
      if (step == 0) currentCommand = new MoveBy(drive, 4);
      //rotate 180
      else if (step == 1) currentCommand = new LimelightTurn(drive, 180, limelight);
      //feedShooter for 5 seconds
      if (step == 2) {
        tube.setTubeMode(TubeMode.feed);
        currentCommand = new TimerCommand(5);
      }
      //stop robot
      else if (step == 3) isFinished = true;
 
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
