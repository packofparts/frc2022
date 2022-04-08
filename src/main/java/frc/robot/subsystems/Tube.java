package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Tube extends SubsystemBase {
  //base motors
  CANSparkMax intakeMotor = new CANSparkMax(IDs.intakeMotorID, MotorType.kBrushless);
  TalonSRX indexMotor = new TalonSRX(IDs.indexMotorID);
  TalonSRX feederMotor = new TalonSRX(IDs.feederMotorID);
  TubeMode tubeMode = TubeMode.off;

  //joysticks
  Joysticks joysticks;

  //pneumatics
  Solenoid intakeSolenoid1;
  Solenoid intakeSolenoid2;
  Compressor phCompressor;
  AnalogPotentiometer pressureSensor;
  PneumaticHub hub;

  boolean useJoysticks = true;
  Timer intakeTimeout = new Timer();
  
  public Tube(Joysticks joysticks) {
    this.joysticks = joysticks;
    intakeMotor.setInverted(true);
    indexMotor.setInverted(true);
    feederMotor.setInverted(false);
 
    indexMotor.configVoltageCompSaturation(12);
    indexMotor.enableVoltageCompensation(true);
    feederMotor.configVoltageCompSaturation(12);
    feederMotor.enableVoltageCompensation(true);
    intakeSolenoid1 = new Solenoid(PneumaticsModuleType.REVPH, IDs.intakeSolenoid1ID);
    intakeSolenoid2 = new Solenoid(PneumaticsModuleType.REVPH, IDs.intakeSolenoid2ID);
    phCompressor = new Compressor(IDs.compressorID, PneumaticsModuleType.REVPH);
    pressureSensor = new AnalogPotentiometer(0, 250, -13);
    hub = new PneumaticHub();

    intakeTimeout.reset();
  }

  public enum TubeMode {
    intake, feed, reverse, off;
  }

  public void setUseJoysticks(boolean joysticks) {
    this.useJoysticks = joysticks;
  }

  @Override
  public void periodic() {
    if (useJoysticks) {
      //set tube mode && run
      if (joysticks.getIntakeToggle()) setTubeMode(TubeMode.intake);
      else if (joysticks.getOutakeToggle()) setTubeMode(TubeMode.reverse);
      else if (joysticks.getIndexToggle()) setTubeMode(TubeMode.feed);
      else setTubeMode(TubeMode.off);
    }

    runTube();

    //compressor management
    if (phCompressor.getPressure() > 119) phCompressor.disable();
    else if (phCompressor.getPressure() < 118) phCompressor.enableDigital();

    //toggle intake solenoids
    if (joysticks.getIntakeSolenoidToggle()) {
      if (intakeSolenoid1.get()) setPneumatics(false);
      else setPneumatics(true);
    }

    SmartDashboard.putNumber("digital pressure", phCompressor.getPressure());
    SmartDashboard.putBoolean("intake pneumatics", intakeSolenoid1.get() && intakeSolenoid2.get());
  }

  public void setTubeMode(TubeMode mode) {
    this.tubeMode = mode;
  }

  public void stopTube() {
    phCompressor.disable();
    tubeMode = TubeMode.off;
    runTube();
  }

  public void runTube() {
    //intake in, index in, feeder out
    if (tubeMode == TubeMode.intake) {
      intakeMotor.set(1);
      indexMotor.set(TalonSRXControlMode.PercentOutput, 0.3);
      feederMotor.set(TalonSRXControlMode.PercentOutput, -0.3);
    }
    //all in
    else if (tubeMode == TubeMode.feed) {// REMOVED STOPPER DUE TO INDEX SLOW TO PUSH BALL
     // if (intakeTimeout.get() == 0) intakeTimeout.start();

     // if (intakeTimeout.get() <= 0.5) {
        intakeMotor.set(1);
        indexMotor.set(TalonSRXControlMode.PercentOutput, 0.5);
        feederMotor.set(TalonSRXControlMode.PercentOutput, 1);
     // }
      // else {
      //   intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
      //   indexMotor.set(TalonSRXControlMode.PercentOutput, 0);
      //   feederMotor.set(TalonSRXControlMode.PercentOutput, 0);
      //   if (intakeTimeout.get() >= 0.6) {
      //     intakeTimeout.stop();
      //     intakeTimeout.reset();
      //   }
      //}
    }
    //all out
    else if (tubeMode == TubeMode.reverse) {
      intakeMotor.set(-1);
      indexMotor.set(TalonSRXControlMode.PercentOutput, -1);
      feederMotor.set(TalonSRXControlMode.PercentOutput, -1);
    }
    else {
      intakeMotor.set(0);
      indexMotor.set(TalonSRXControlMode.PercentOutput, 0);
      feederMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
    
    SmartDashboard.putBoolean("Intake Running", intakeMotor.get() != 0);
    SmartDashboard.putBoolean("Index Running", indexMotor.getMotorOutputPercent() != 0);
    SmartDashboard.putBoolean("Feeder Running", feederMotor.getMotorOutputPercent() != 0);
  }

  public void setPneumatics(boolean extend) {
    intakeSolenoid1.set(extend);
    intakeSolenoid2.set(extend);
  }
}