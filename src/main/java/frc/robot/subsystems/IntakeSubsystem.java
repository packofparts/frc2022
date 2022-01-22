// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.awt.Color;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Constants.intakeColorSensor.getColor() == edu.wpi.first.wpilibj.util.Color.kBlue && Constants.xController.getAButton()){
      Constants.intakeMotor1.set(-1);
      Constants.intakeMoror2.set(1);
    } else if (Constants.xController.getAButton()){
        Constants.intakeMotor1.set(1);
        Constants.intakeMoror2.set(-1);
    } else if (Constants.xController.getBButton()) {
        Constants.intakeMotor1.set(-1);
        Constants.intakeMoror2.set(1);
    } else {
        Constants.intakeMotor1.stopMotor();
        Constants.intakeMoror2.stopMotor();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
