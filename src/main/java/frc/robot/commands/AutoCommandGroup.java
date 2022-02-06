// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;
public class AutoCommandGroup extends ParallelCommandGroup {
  /** Add your docs here. */

  // IntakeSubsystem intake = new IntakeSubsystem();
  DriveSubsystem drive = new DriveSubsystem();
  IntakeSubsystem intake = new IntakeSubsystem();
  Index index = new Index();
  //Shooter shooter = new Shooter();


  public AutoCommandGroup() {
    //Step 1- Turn 180
    parallel(new PIDTurn(drive, 180));

    //Step 2- Move 116.82 inches or 9.735 ft
    parallel(new MoveBy(drive, 9.735));

    //Step 3- Intake Ball
    parallel(new IntakeBall(intake));

    //Step 4- Turn 180
    parallel(new PIDTurn(drive, 180));

    //Step 5- Fire Ball 1- Cannot Complete without shooter
    //parallel(new ShootBall());

    //Step 6- Transfer Index to Shooter
    parallel(new transferIndex(Constants.indexSpeed, index));

    //Step 7- Fire Ball 2- Cannot Complete without shooter
    //parallel(new ShootBall());
    parallel(new limelightMove(drive));
    if(Constants.AllianceIsRed){
      if (Constants.tc[0]<=Constants.targetValuesR[0]+10 && Constants.tc[0]>=Constants.targetValuesR[0]-10) {
        if (Constants.tc[1]<=Constants.targetValuesR[1]+10 && Constants.tc[1]>=Constants.targetValuesR[1]-10) {
          if (Constants.tc[2]<=Constants.targetValuesR[2]+10 && Constants.tc[2]>=Constants.targetValuesR[2]-10) {
            parallel(new IntakeBall(intake));
    }}}}

    
    if (Constants.AllianceIsRed){
      if (Constants.tc[0]<=Constants.targetValuesB[0]+10 && Constants.tc[0]>=Constants.targetValuesB[0]-10) {
        if (Constants.tc[1]<=Constants.targetValuesB[1]+10 && Constants.tc[1]>=Constants.targetValuesB[1]-10) {
          if (Constants.tc[2]<=Constants.targetValuesB[2]+10 && Constants.tc[2]>=Constants.targetValuesB[2]-10) {
            parallel(new PIDTurn(drive,90));
            while(Constants.tv = false){
              parallel(new PIDTurn(drive,360));
            }
            parallel(new limelightMove(drive));
    }}}}

    if(Constants.AllianceIsRed==false){
      if (Constants.tc[0]<=Constants.targetValuesB[0]+10 && Constants.tc[0]>=Constants.targetValuesB[0]-10) {
        if (Constants.tc[1]<=Constants.targetValuesB[1]+10 && Constants.tc[1]>=Constants.targetValuesB[1]-10) {
          if (Constants.tc[2]<=Constants.targetValuesB[2]+10 && Constants.tc[2]>=Constants.targetValuesB[2]-10) {
            parallel(new IntakeBall(intake));
    }}}}

    //detect blue
    if (Constants.AllianceIsRed==false){
      if (Constants.tc[0]<=Constants.targetValuesR[0]+10 && Constants.tc[0]>=Constants.targetValuesR[0]-10) {
        if (Constants.tc[1]<=Constants.targetValuesR[1]+10 && Constants.tc[1]>=Constants.targetValuesR[1]-10) {
          if (Constants.tc[2]<=Constants.targetValuesR[2]+10 && Constants.tc[2]>=Constants.targetValuesR[2]-10) {
            parallel(new PIDTurn(drive,90));
            while(Constants.tv = false){
              parallel(new PIDTurn(drive,360));
            }
            parallel(new limelightMove(drive));
    }}}}

    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
