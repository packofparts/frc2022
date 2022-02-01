package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight extends SubsystemBase {
  /** Creates a new limelight. */
  PIDController pidController;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry targetx = table.getEntry("tx");
  double tx = targetx.getDouble(0);
  DriveSubsystem driveBase;

  public Limelight(DriveSubsystem drive) {
    this.driveBase = drive;
    this.pidController = new PIDController(Constants.turnPID[0], Constants.turnPID[1], Constants.turnPID[2]);
    pidController.setSetpoint(0);
  }
  
  public void periodic() {
    // Move outside if error
    // if (joysticks.getRawButton(0)) {
    //   double speed = pidController.calculate(this.tx);
    //   driveBase.m_backLeftSpark.set(-speed);
    //   driveBase.m_frontLeftSpark.set(-speed);
    //   driveBase.m_backRightSpark.set(speed);
    //   driveBase.m_frontRightSpark.set(speed);
    // }
  }

  public double getTx() {
    return tx;
  }
}
