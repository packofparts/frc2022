package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  public static Talon intakeMotor1 = new Talon(Constants.intakeMotor1Port);
  public static Talon intakeMoror2 = new Talon(Constants.intakeMotor2Port);
  public static ColorSensorV3 intakeColorSensor = new ColorSensorV3(Constants.intakeColorSensorPort); 
  public static Ultrasonic intakeUltrasonic = new Ultrasonic(Constants.intakeUltrasonicPingPort, Constants.intakeUltrasonicEchoPort);


  
  /** Creates a new Intake. */
  public intake() {
    exampleSolenoidPH.set(true);
  }

  @Override
    public void periodic() {
    // This method will be called once per scheduler run
    if (intakeColorSensor.getColor() == edu.wpi.first.wpilibj.util.Color.kBlue && joystick.getRawButton(1)){
      intakeMotor1.set(-1);
      intakeMoror2.set(1);
    } else if (joystick.getRawButton(1)){
        intakeMotor1.set(1);
        intakeMoror2.set(-1);
    } else if (joystick.getRawButton(2)) {
        intakeMotor1.set(-1);
        intakeMoror2.set(1);
    } else if (!Robot.m_robotContainer.m_index.hasBall()) {
        intakeMotor1.set(1);
        intakeMoror2.set(-1);
    } else {
        intakeMotor1.stopMotor();
        intakeMoror2.stopMotor();
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

    

    SmartDashboard.putString("Color Sensor", intakeColorSensor.getColor().toString());
    SmartDashboard.putNumber("Joystick POV", joystick.getPOV());
    SmartDashboard.putNumber("Joystick Axis", _joystick.getRawAxis(2));
    SmartDashboard.putNumber("Ultrasonic Sensor", intakeUltrasonic.getRangeInches());

  }

  public double getUltrasonicInches() {
    return intakeUltrasonic.getRangeInches();
  }

  public void setRPM(double RPM) {
    intakeMotor.set(RPM);
  }



}