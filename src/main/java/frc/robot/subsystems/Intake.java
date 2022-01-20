@@ -5,55 +5,31 @@
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class intake extends SubsystemBase {
  Solenoid exampleSolenoidPH = new Solenoid(PneumaticsModuleType.REVPH, 1);
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  double current = phCompressor.getPressure(); //no idea what this does, probably an issue
  XboxController joystick = new XboxController(0);
  Joystick _joystick = new Joystick(0);
  TalonFX intakeMotor = new TalonFX(0);
  //PIDSetConfiguration
  double RPM=0.0;
  boolean isXbox = true;
  phCompressor.enableDigital();


  exampleSolenoidPH.set(true);
  /** Creates a new Intake. */
  public intake() {  }

  @Override
    public void periodic() {
    // This method will be called once per scheduler run
    if (//joystick thing whenever i can download) { //pushes forward
      exampleSolenoidPH.toggle();
    }

    else {

      double stick = _joystick.getRawAxis(2);
      intakeMotor.set(ControlMode.Velocity, -stick);
    if (//joystick thing whenever i can download) { //pulls back
      exampleSolenoidPH.toggle();
    }
    if(isXbox){
      if (joystick.getPOV() == 0) {
        if (RPM+Constants.increment<=7500) {
          setRPM(RPM+Constants.increment);
        }
      }
      if (joystick.getPOV() == 180) {
        if (RPM-Constants.increment>=0) {
          setRPM(RPM-Constants.increment);
        }
      }
    }

    else {

      double stick = _joystick.getRawAxis(2);
      intakeMotor.set(ControlMode.Velocity, -stick);
    }
  }

  public void setRPM(double RPM) {
    intakeMotor.set(ControlMode.Velocity, RPM);
  }

}