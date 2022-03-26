// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.lang.Math;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.LimelightTurn;
import frc.robot.commands.MoveBy;
import frc.robot.subsystems.Limelight;
import frc.robot.constants.Constants;
import frc.robot.constants.IDs;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax.ControlType;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.MoveBy;
public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  Joysticks joysticks;
  Limelight lime = new Limelight();
  TalonFX mainTalon = new TalonFX(IDs.flyWheelID);
  TalonFX rollerTalon = new TalonFX(IDs.rollerID);
  DriveSubsystem drivebase = new DriveSubsystem(joysticks, lime);
  int shooterModeIndex = 0;
  ShooterMode shooterMode = ShooterMode.off;
  boolean shooterReady = false;
  boolean shooterScrollPressed = false;
  double setVelocityMain = 0;
  double setVelocityRoller = 0;

  final boolean tuningRPM = false;
  final ShooterMode[] modes = new ShooterMode[] 
  {ShooterMode.normal, ShooterMode.launchPadFar, ShooterMode.launchPadNear, ShooterMode.tarmac, ShooterMode.closeLow};
  
  public Shooter(Joysticks joysticks) {
    this.joysticks = joysticks;

    mainTalon.config_kP(0, Constants.shooterMainPID[0]);
    mainTalon.config_kI(0, Constants.shooterMainPID[1]);
    mainTalon.config_kD(0, Constants.shooterMainPID[2]);
    mainTalon.config_kF(0, Constants.shooterMainPID[3]);
    
    rollerTalon.config_kP(0, Constants.shooterRollerPID[0]);
    rollerTalon.config_kI(0, Constants.shooterRollerPID[1]);
    rollerTalon.config_kD(0, Constants.shooterRollerPID[2]);
    rollerTalon.config_kF(0, Constants.shooterRollerPID[3]);

    if (tuningRPM) {
      SmartDashboard.putNumber("mainRoll", 0);
      SmartDashboard.putNumber("secondRoll", 0); 
    }
  }

  public enum ShooterMode {
    normal, launchPadNear, launchPadFar, tarmac, closeLow, auto, ballReject, off, quickShot, autoTrajectory;
  }


  @Override
  public void periodic() {
    // shooter toggle
    if (joysticks.getShooterToggle()) {
      if (shooterMode != ShooterMode.off) setShooterMode(ShooterMode.off);
      else setShooterMode(modes[shooterModeIndex]);
    }

    if (joysticks.getShooterScrollDown()){
      setShooterMode(ShooterMode.ballReject);
    }

    if (joysticks.getNotPOV()) {
      shooterScrollPressed = false;
    }
    if (joysticks.getShooterScrollRight() && !shooterScrollPressed){
      if (shooterModeIndex < modes.length -1){
        shooterModeIndex ++;
      }
      shooterScrollPressed = true;
    }
    else if (joysticks.getShooterScrollLeft() && !shooterScrollPressed){
      if (shooterModeIndex > 0){
        shooterModeIndex --;
      }
      shooterScrollPressed = true;
    }
   
    SmartDashboard.putString("Selected Shooter Mode", modes[shooterModeIndex] + "");

    //shooter mode
    runShooter();
  }

  public boolean getShooterReady() {
    return shooterReady;
  }
  
  public void setShooterMode(ShooterMode mode) {
    this.shooterMode = mode;
  }

  public void stopShooter() {
    this.shooterMode = ShooterMode.off;
    runShooter();
  }

  public void runShooter() {
    if (tuningRPM) {
      mainTalon.set(TalonFXControlMode.PercentOutput, SmartDashboard.getNumber("mainRoll", 0));
      rollerTalon.set(TalonFXControlMode.PercentOutput, SmartDashboard.getNumber("secondRoll", 0));
      
      SmartDashboard.putNumber("Shooter-Main", mainTalon.getSelectedSensorVelocity());
      SmartDashboard.putNumber("Shooter-Roller", rollerTalon.getSelectedSensorVelocity());
      return;
    }
    else if (shooterMode == ShooterMode.normal) {
      setVelocityMain = 5200;//6400;
      setVelocityRoller = -4600;//-4200;
    }
    else if (shooterMode == ShooterMode.launchPadFar) {
      setVelocityMain = 7000;
      setVelocityRoller = -6000;
    }
    else if (shooterMode == ShooterMode.launchPadNear) {
      setVelocityMain = 6400;
      setVelocityRoller = -4200;
    }
    else if (shooterMode == ShooterMode.closeLow) {
      setVelocityMain = 6600;
      setVelocityRoller = -4500;
    }
    else if (shooterMode == ShooterMode.tarmac) {
      setVelocityMain = 4900;
      setVelocityRoller = -4700;
    }
    else if (shooterMode == ShooterMode.auto) {
      setVelocityMain = 4900;
      setVelocityRoller = -4700;
    }
    else if (shooterMode == ShooterMode.ballReject) {
      setVelocityMain = 2000;
      setVelocityRoller = -2000;
    }
    else if(shooterMode ==  ShooterMode.quickShot){
      // get distance from hub
     double distance = lime.getHubDist();
     double ShotRPM = 1727;//MAKE ADVANCED LINEAR REGRESSION ALGORITHM HERE
      setVelocityMain = ShotRPM;
      setVelocityRoller= ShotRPM;
    }
    else if (shooterMode == ShooterMode.autoTrajectory){
      double distance = lime.getHubDist();
      // 7 intervals initially
      //rpm values are [setVelocityMain, setVelocityRoller]
      int[][] rpmValues = ;
      double[] distanceTrajectoryLength = ;
      int indexMin = 99999;
      double moveWhere = 0.0;
      double minDist = 999999999999.99;
      for(int i = 0; i< rpmValues.length; i++){
        double currDist = Math.abs(distance - distanceTrajectoryLength[i]);
        if(currDist < minDist){
          minDist = currDist;
          indexMin = i;
        }
      }
      moveWhere = distanceTrajectoryLength[indexMin];
      setVelocityMain = rpmValues[indexMin,0]; 
      setVelocityRoller = rpmValues[indexMin,1];
      drivebase.drive(moveWhere, 0.0, 0.0, true);
    }
    else {
      setVelocityMain = 0;
      setVelocityRoller = 0;
    }

    if (setVelocityMain == 0) mainTalon.set(TalonFXControlMode.PercentOutput, 0);
    else mainTalon.set(TalonFXControlMode.Velocity, setVelocityMain);

    if (setVelocityRoller == 0) rollerTalon.set(TalonFXControlMode.PercentOutput, 0);
    else rollerTalon.set(TalonFXControlMode.Velocity, setVelocityRoller);

    shooterReady = Math.abs(Math.abs(setVelocityMain)-Math.abs(mainTalon.getSelectedSensorVelocity())) <= Constants.shooterDeadzone &&
    Math.abs(Math.abs(setVelocityRoller)-Math.abs(rollerTalon.getSelectedSensorVelocity())) <= Constants.shooterDeadzone && 
                    setVelocityMain != 0 && setVelocityRoller != 0;

    SmartDashboard.putBoolean("Shooter Running", mainTalon.getMotorOutputPercent() != 0 || rollerTalon.getMotorOutputPercent() != 0);
    // SmartDashboard.putBoolean("Shooter Ready", shooterReady);

    if (shooterReady) joysticks.rumbleOperator(1);
    else joysticks.rumbleOperator(0);

    SmartDashboard.putString("Active Shooter Mode", shooterMode + "");
    SmartDashboard.putNumber("Shooter-Main", mainTalon.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter-Roller", rollerTalon.getSelectedSensorVelocity());
  }
}