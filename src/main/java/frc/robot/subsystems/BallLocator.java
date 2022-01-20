package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallLocator extends SubsystemBase {

  Drive drive;

  PIDController pid = new PIDController(0, 0, 0);

  public BallLocator(Drive drive) {
    this.drive = drive;
  }

  @Override
  public void periodic() {
    //1) Turn 180 degrees
    pid.calculate(drive.yaw(), 180);

    //2) Go till within disFromBallErrorMargin distance


    //3) Go to predetermined distance


    //4) Call shooter
    
  }
}
