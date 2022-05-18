// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class PlayMusic extends CommandBase {
  Orchestra musik;
  /** Creates a new PlayMusic. */
  public PlayMusic() {
    // Use addRequirements() here to declare subsystem dependencies.
    musik = new Orchestra();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
    Robot.m_robotContainer.shooter.isMusicPlaying = true;
    musik.loadMusic("starwarsegm1*******************************=====================================================================================================================================================================================================================================-==========================================--0-=0000].chrp");
    System.out.println("Now playing music!");
    ErrorCode result = musik.addInstrument(Robot.m_robotContainer.shooter.mainTalon);
    ErrorCode result2 = musik.addInstrument(Robot.m_robotContainer.shooter.rollerTalon);
    if(result != ErrorCode.OK) {
      System.out.println("You dum dum, error code was " + result +  result.toString());
    }
    musik.play();
    if(result2 != ErrorCode.OK) {
      System.out.println("You dum dum, error code was " + result2 +  result2.toString());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_robotContainer.shooter.isMusicPlaying = false;
    musik.stop();
    musik.clearInstruments();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean endy = Robot.m_robotContainer.joysticks.getEndMusicButton();
    if(endy) System.out.println("Music Ending");
    return endy;
  }
}
