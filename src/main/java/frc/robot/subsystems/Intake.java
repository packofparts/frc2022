// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;


public class Intake extends SubsystemBase {
  Solenoid exampleSolenoidPH = new Solenoid(PneumaticsModuleType.REVPH, 1);
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  double current = phCompressor.getPressure(); //no idea what this does, probably an issue

  exampleSolenoidPH.set(true);
  /** Creates a new Intake. */
  public Intake() {
    if (//joystick thing whenever i can download) { //pushes forward
      exampleSolenoidPH.toggle();
    }
    if (//joystick thing whenever i can download) { //pulls back
      exampleSolenoidPH.toggle();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
