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
  Talon intakeMotor = new Talon(0);
  //PIDSetConfiguration
  double RPM=0.0;
  boolean isXbox = true;


  
  /** Creates a new Intake. */
  public intake() {
    exampleSolenoidPH.set(true);
  }

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


    if(isXbox){
      if (joystick.getPOV() == 0) {
        exampleSolenoidPH.toggle();
        if (RPM+Constants.increment<=7500) {
          setRPM(RPM+Constants.increment);
        }
      }
      if (joystick.getPOV() == 180) {
        exampleSolenoidPH.toggle();
        if (RPM-Constants.increment>=0) {
          setRPM(RPM-Constants.increment);
        }
      }
    }

    else {
      double stick = _joystick.getRawAxis(2);
      intakeMotor.set(-stick);
    }
  }

  public void setRPM(double RPM) {
    intakeMotor.set(RPM);
  }

}