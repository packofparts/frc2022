// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.*;

public class AutoCommandGroup extends ParallelCommandGroup {
  /** Add your docs here. */

  // IntakeSubsystem intake = new IntakeSubsystem();
  //Shooter shooter = new Shooter();


  public AutoCommandGroup(DriveSubsystem drive, Intake intake, Index index) {
    //Step 1- Turn 180
    parallel(new PIDTurn(drive, 180));

    //Step 2- Move 116.82 inches or 9.735 ft
    parallel(new MoveBy(drive, 9.735));

    //Step 3- Intake Ball
    // parallel(new IntakeBall(intake));

    //Step 4- Turn 180
    parallel(new PIDTurn(drive, 180));

    //Step 5- Fire Ball 1- Cannot Complete without shooter
    //parallel(new ShootBall());

    //Step 6- Transfer Index to Shooter
    parallel(new transferIndex(index, 0.3));

    //Step 7- Fire Ball 2- Cannot Complete without shooter
    //parallel(new ShootBall());
  }
}
