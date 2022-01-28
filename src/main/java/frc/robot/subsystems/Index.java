// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Index extends SubsystemBase {
  public static Ultrasonic indexUltrasonic1 = new Ultrasonic(Constants.indexUltrasonic1PingPort, Constants.indexUltrasonic1EchoPort);
  public static Ultrasonic indexUltrasonic2 = new Ultrasonic(Constants.indexUltrasonic2PingPort, Constants.indexUltrasonic2EchoPort);
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
    if(Robot.m_robotContainer.m_intake.getUltrasonicInches() < Constants.ultrasonicThreshold) {
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
    SmartDashboard.putNumber("Index Ultrasonic 1 ", indexUltrasonic1.getRangeInches());
    SmartDashboard.putNumber("Index Ultrasonic 2 ", indexUltrasonic2.getRangeInches());
  }

  public boolean hasBall() {
    return Robot.m_robotContainer.m_intake.getUltrasonicInches() < Constants.ultrasonicThreshold;
  }
}
