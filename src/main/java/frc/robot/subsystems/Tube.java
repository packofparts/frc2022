package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IDs;
import frc.robot.subsystems.Limelight.Pipeline;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Tube extends SubsystemBase {
  //base motors
  TalonSRX intakeMotor = new TalonSRX(IDs.intakeMotorID);
  TalonSRX indexMotor = new TalonSRX(IDs.indexMotorID);
  TalonSRX feederMotor = new TalonSRX(IDs.feederMotorID);
  TubeMode tubeMode = TubeMode.off;

  //joysticks
  Joysticks joysticks;

  //pneumatics
  final boolean usePneumatics = true;
  Solenoid intakeSolenoid1;
  Solenoid intakeSolenoid2;
  Compressor phCompressor;
  AnalogPotentiometer pressureSensor;
  PneumaticHub hub;

  //sensors
  final boolean useSensors = false;
  ColorSensorV3 intakeColorSensor;
  Pipeline intakeBall = Pipeline.none;

  AnalogInput indexUltrasonicFront;
  boolean indexFront = false;

  AnalogInput indexUltrasonicBack;
  boolean indexBack = false;

  boolean useJoysticks = true;
  Timer intakeTimeout = new Timer();
  
  public Tube(Joysticks joysticks) {
    this.joysticks = joysticks;
    indexMotor.setInverted(true);
    feederMotor.setInverted(false);
 
    if (usePneumatics) {
      intakeSolenoid1 = new Solenoid(PneumaticsModuleType.REVPH, IDs.intakeSolenoid1ID);
      intakeSolenoid2 = new Solenoid(PneumaticsModuleType.REVPH, IDs.intakeSolenoid2ID);
      phCompressor = new Compressor(IDs.compressorID, PneumaticsModuleType.REVPH);
      pressureSensor = new AnalogPotentiometer(0, 250, -13);
      hub = new PneumaticHub();
    }
    if (useSensors) {
      intakeColorSensor = new ColorSensorV3(IDs.intakeColorSensorPort); 
      indexUltrasonicFront = new AnalogInput(IDs.intakeUltrasonicFront);
      indexUltrasonicBack = new AnalogInput(IDs.intakeUltrasonicBack);
    }

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

    //pneumatics
    if (usePneumatics) {
      //compressor management
      if (phCompressor.getPressure() > 112) phCompressor.disable();
      else if (phCompressor.getPressure() < 110) phCompressor.enableDigital();

      //toggle intake solenoids
      if (joysticks.getIntakeSolenoidToggle()) {
        if (intakeSolenoid1.get()) setPneumatics(false);
        else setPneumatics(true);
      }

      SmartDashboard.putNumber("digital pressure", phCompressor.getPressure());
      SmartDashboard.putBoolean("intake pneumatics", intakeSolenoid1.get() && intakeSolenoid2.get());
    }

    //sensors
    if (useSensors) {
      //colorsensor rawcolor for debugging
      double r = (double)intakeColorSensor.getRed()/intakeColorSensor.getProximity();
      double g = (double)intakeColorSensor.getGreen()/intakeColorSensor.getProximity();
      double b = (double)intakeColorSensor.getBlue()/intakeColorSensor.getProximity();

      //intake color sensor (only if within range)
      if (intakeColorSensor.getProximity() > Constants.minColorSensorDis) {
        //red ball threshold
        if (r > b) intakeBall = Pipeline.red;
        //blue ball threshold
        else if (b > r) intakeBall = Pipeline.blue;
        else intakeBall = Pipeline.unknown;
      }
      else intakeBall = Pipeline.none;

      //index ultrasonic front
      if (indexUltrasonicFront.getValue() < Constants.indexUltrasonicFrontThreshold) indexFront = true;
      else indexFront = false;

      //index ultrasonic back
      if (indexUltrasonicBack.getValue() < Constants.indexUltrasonicBackThreshold) indexBack = true;
      else indexBack = false;

      SmartDashboard.putString("ColorSensor", r + " , " + g + " , " + b);
      
      SmartDashboard.putBoolean("Intake Ball", intakeBall != Pipeline.none);
      SmartDashboard.putString("Intake Ball Color", intakeBall + "");
      SmartDashboard.putBoolean("Index Front", indexFront);
      SmartDashboard.putBoolean("Index Back", indexBack);
      SmartDashboard.putNumber("Intake Color Sensor Distance", intakeColorSensor.getProximity());
      SmartDashboard.putNumber("UltraFront", indexUltrasonicFront.getValue());
      SmartDashboard.putNumber("UltraBack", indexUltrasonicBack.getValue());
      SmartDashboard.putBoolean("IsBallFront", indexUltrasonicFront.getValue() < 430);
      SmartDashboard.putBoolean("IsBallBack", indexUltrasonicBack.getValue() < 430);
    }
  }

  public boolean getIntake() {
    return intakeBall != Pipeline.none;
  }

  public boolean getIntake(Pipeline pipeline) {
    return intakeBall == pipeline;
  }

  public boolean getIndexFront() {
    return indexFront;
  }

  public boolean getIndexBack() {
    return indexBack;
  }

  public void setTubeMode(TubeMode mode) {
    this.tubeMode = mode;
  }

  public void stopTube() {
    if (usePneumatics) {
      phCompressor.disable();
    }

    tubeMode = TubeMode.off;
    runTube();
  }

  public void runTube() {
    //intake in, index in, feeder out
    if (tubeMode == TubeMode.intake) {
      intakeMotor.set(TalonSRXControlMode.PercentOutput, 1);
      indexMotor.set(TalonSRXControlMode.PercentOutput, 0);
      feederMotor.set(TalonSRXControlMode.PercentOutput, -0.2);
    }
    //all in
    else if (tubeMode == TubeMode.feed) {// REMOVED STOPPER DUE TO INDEX SLOW TO PUSH BALL
     // if (intakeTimeout.get() == 0) intakeTimeout.start();

     // if (intakeTimeout.get() <= 0.5) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 1);
        indexMotor.set(TalonSRXControlMode.PercentOutput, .5);
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
      intakeMotor.set(TalonSRXControlMode.PercentOutput, -1);
      indexMotor.set(TalonSRXControlMode.PercentOutput, -1);
      feederMotor.set(TalonSRXControlMode.PercentOutput, -1);
    }
    else {
      intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
      indexMotor.set(TalonSRXControlMode.PercentOutput, 0);
      feederMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
    
    SmartDashboard.putBoolean("Intake Running", intakeMotor.getMotorOutputPercent() != 0);
    SmartDashboard.putBoolean("Index Running", indexMotor.getMotorOutputPercent() != 0);
    SmartDashboard.putBoolean("Feeder Running", feederMotor.getMotorOutputPercent() != 0);
  }

  public void setPneumatics(boolean extend) {
    if (!usePneumatics) return;
    
    intakeSolenoid1.set(extend);
    intakeSolenoid2.set(extend);
  }
}