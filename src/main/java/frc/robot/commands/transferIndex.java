// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.constants.Constants;
// import frc.robot.subsystems.Index;
// import edu.wpi.first.wpilibj.Timer;
// public class transferIndex extends CommandBase {
//   /** Creates a new transferIndex. */

//   Index index;
//   Timer timer;
//   boolean finish = false;
//   double indexSpeed;

//   public transferIndex(Index in, double indexSpeed) {
//     this.index = in;
//     this.indexSpeed = indexSpeed;

//     timer = new Timer();
//     addRequirements(index);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     timer.reset();
//     timer.start();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (timer.get() < Constants.spinTime) {
//       index.setIndex(indexSpeed);
//     }
//     else {
//       finish = true;
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     System.out.println("index complete");
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return finish;
//   }
// }
