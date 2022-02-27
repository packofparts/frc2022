package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IDs;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Tube extends SubsystemBase {
  // Solenoid intakeSolenoid1 = new Solenoid(PneumaticsModuleType.REVPH, IDs.intakeSolenoid1ID);
  // Solenoid intakeSolenoid2 = new Solenoid(PneumaticsModuleType.REVPH, IDs.intakeSolenoid2ID);
  // Compressor phCompressor = new Compressor(IDs.compressorID, PneumaticsModuleType.REVPH);
  // AnalogPotentiometer pressureSensor = new AnalogPotentiometer(0, 250, -13);
  // PneumaticHub hub = new PneumaticHub();

  //PIDSetConfiguration
  double RPM=0.0;
  boolean isXbox = true;
  TalonSRX intakeMotor = new TalonSRX(IDs.intakeMotor1ID);
  // Talon intakeMotor2 = new Talon(IDs.intakeMotor2ID);
  // ColorSensorV3 intakeColorSensor = new ColorSensorV3(IDs.intakeColorSensorPort); 
  // Ultrasonic intakeUltrasonic = new Ultrasonic(IDs.intakeUltrasonicPingPort, IDs.intakeUltrasonicEchoPort);

  TalonSRX indexMotor = new TalonSRX(IDs.indexMotorID);

  Timer timer = new Timer();

  Joysticks joysticks;
  
  /** Creates a new Intake. */
  public Tube(Joysticks joysticks) {
    this.joysticks = joysticks;
    // intakeSolenoid1.set(false);
    // intakeSolenoid2.set(false);
  }

  @Override
    public void periodic() {
      // if (phCompressor.getPressure() > 62) phCompressor.disable();
      // else if (phCompressor.getPressure() < 60) phCompressor.enableDigital();
    // This method will be called once per scheduler run
    // if (intakeColorSensor.getColor() == edu.wpi.first.wpilibj.util.Color.kBlue && joystick.getRawButton(1)){
    //   intakeMotor1.set(-1);
    //   intakeMoror2.set(1);
    // }

    // SmartDashboard.putNumber("digital pressure", phCompressor.getPressure());
    // SmartDashboard.putNumber("analog current", phCompressor.getCurrent());
    // SmartDashboard.putNumber("analog voltage", phCompressor.getAnalogVoltage());
    // System.out.println(phCompressor.getPressure());
    // System.out.println(phCompressor.getAnalogVoltage());
    // System.out.println(hub.getPressure(0));
    
    //intake
    if (joysticks.getIntakeToggle()){
      System.out.println("intake");
      intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.5);
      // intakeMotor2.set(-1);
    } 
    //outtake
    else if (joysticks.getOutakeToggle()) {
      System.out.println("outtake");
      intakeMotor.set(TalonSRXControlMode.PercentOutput, -0.5);
      // intakeMotor2.set(1);
    }
    else {
      intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    if (joysticks.getIntakeSolenoidToggle()) {
      // toggleMatics();
    }

    if (joysticks.getIndexToggle()) indexMotor.set(TalonSRXControlMode.PercentOutput, -1);
    else if (joysticks.getOutdexToggle()) indexMotor.set(TalonSRXControlMode.PercentOutput, 1);
    else indexMotor.set(TalonSRXControlMode.PercentOutput, 0);

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

    // SmartDashboard.putString("Color Sensor Color", intakeColorSensor.getColor().toString());
    // SmartDashboard.putNumber("Color Sensor Distance", intakeColorSensor.getProximity());
    // // SmartDashboard.putNumber("Joystick POV", joystick.getPOV());
    // SmartDashboard.putNumber("Ultrasonic Sensor", intakeUltrasonic.getRangeInches());

  }

  // public double getUltrasonicInches() {
  //   return intakeUltrasonic.getRangeInches();
  // }

  // public void intakeBall() {
  //   timer.reset();
  //   timer.start();
  //   if (timer.get() < Constants.spinTimeIntake) {
  //     // intakeMotor.set(1);
  //     // intakeMotor2.set(-1);
  //   }
  // }

  // public void setSolenoidPH(boolean val) {
  //   intakeSolenoid1.set(val);
  //   intakeSolenoid2.set(val);
  // }

  // public void outtakeBall() {
  //   timer.reset();
  //   timer.start();
  //   if (timer.get() < Constants.spinTimeOuttake) {
  //     // intakeMotor.set(-1);
  //     // intakeMotor2.set(1);
  //   }
  // }
  
  // public void toggleMatics(){
  //   intakeSolenoid1.toggle();
  //   intakeSolenoid2.toggle();
  // }
}