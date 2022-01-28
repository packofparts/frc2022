// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Index extends SubsystemBase {
  Talon indexMotor = new Talon(Constants.indexMotor);
  boolean isIndexing;
  public boolean spinShooter;

  /** Creates a new Index. */
  public Index() {
    isIndexing = false;
    spinShooter = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Robot.m_robotContainer.m_intake.getUltrasonicInches() > 5) {
      isIndexing = true;
      spinShooter = false;
    } else {
      isIndexing = false;
      spinShooter = true;
    }

    if(isIndexing) {
      //Index the index
      indexMotor.set(0.2);
    } else {
      //Don't index the index
      indexMotor.set(0);
    }

  }

  public boolean hasBall() {
    return Robot.m_robotContainer.m_intake.getUltrasonicInches() < 5;
  }
}
