package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;


public class Limelight extends SubsystemBase {
  /** Creates a new limelight. */
  PIDController pidController;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry targetx = table.getEntry("tx");
  public double tx = targetx.getDouble(0);
  DriveSubsystem driveBase = new DriveSubsystem();
  XboxController joystick = new XboxController(0);
  public Limelight() {
    this.pidController = new PIDController(Constants.kpLime, Constants.kiLime, Constants.kdLime);
    pidController.setSetpoint(0);
  }
  
  public void periodic() {
    // Move outside if error
    if (joystick.getRawButton(Constants.autoAlignButton)) {
      double speed = pidController.calculate(this.tx);
      driveBase.m_backLeftSpark.set(-speed);
      driveBase.m_frontLeftSpark.set(-speed);
      driveBase.m_backRightSpark.set(speed);
      driveBase.m_frontRightSpark.set(speed);
    }
  }
}
