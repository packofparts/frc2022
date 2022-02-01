package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IDs;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {
  Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, IDs.intakeSolenoidID);
  Compressor phCompressor = new Compressor(IDs.compressorID, PneumaticsModuleType.REVPH);
  double current = phCompressor.getPressure(); //no idea what this does, probably an issue

  //PIDSetConfiguration
  double RPM=0.0;
  boolean isXbox = true;
  Talon intakeMotor1 = new Talon(IDs.intakeMotor1ID);
  Talon intakeMotor2 = new Talon(IDs.intakeMotor2ID);
  ColorSensorV3 intakeColorSensor = new ColorSensorV3(IDs.intakeColorSensorPort); 
  Ultrasonic intakeUltrasonic = new Ultrasonic(IDs.intakeUltrasonicPingPort, IDs.intakeUltrasonicEchoPort);

  Timer timer = new Timer();

  Joysticks joysticks;
  
  /** Creates a new Intake. */
  public Intake(Joysticks joysticks) {
    this.joysticks = joysticks;
    intakeSolenoid.set(true);
  }

  @Override
    public void periodic() {
    // This method will be called once per scheduler run
    // if (intakeColorSensor.getColor() == edu.wpi.first.wpilibj.util.Color.kBlue && joystick.getRawButton(1)){
    //   intakeMotor1.set(-1);
    //   intakeMoror2.set(1);
    // }
    
    //intake
    if (joysticks.getIntakeToggle()){
      intakeMotor1.set(1);
      intakeMotor2.set(-1);
    } 
    //outtake
    else if (joysticks.getOutakeToggle()) {
      intakeMotor1.set(-1);
      intakeMotor2.set(1);
    }

    // } else if (!Robot.m_robotContainer.m_index.hasBall()) {
    //     intakeMotor1.set(1);
    //     intakeMoror2.set(-1);
    // } else {
    //     intakeMotor1.stopMotor();
    //     intakeMoror2.stopMotor();
    // }


    // if (joystick.getPOV() == 0) {
    //   exampleSolenoidPH.toggle();
    //   if (RPM+Constants.increment<=7500) {
    //     setRPM(RPM+Constants.increment);
    //   }
    // }
    // if (joystick.getPOV() == 180) {
    //   exampleSolenoidPH.toggle();
    //   if (RPM-Constants.increment>=0) {
    //     setRPM(RPM-Constants.increment);
    //   }
    // }

    SmartDashboard.putString("Color Sensor Color", intakeColorSensor.getColor().toString());
    SmartDashboard.putNumber("Color Sensor Distance", intakeColorSensor.getProximity());
    // SmartDashboard.putNumber("Joystick POV", joystick.getPOV());
    SmartDashboard.putNumber("Ultrasonic Sensor", intakeUltrasonic.getRangeInches());

  }

  public double getUltrasonicInches() {
    return intakeUltrasonic.getRangeInches();
  }

  public void intakeBall() {
    timer.reset();
    timer.start();
    if (timer.get() < Constants.spinTimeIntake) {
      intakeMotor1.set(1);
      intakeMotor2.set(-1);
    }
  }

  public void setSolenoidPH(boolean val) {
    intakeSolenoid.set(val);

  }

  public void outtakeBall() {
    timer.reset();
    timer.start();
    if (timer.get() < Constants.spinTimeOuttake) {
      intakeMotor1.set(-1);
      intakeMotor2.set(1);
    }
  }
  
  public void toggleMatics(){
    intakeSolenoid.toggle();
  }
}