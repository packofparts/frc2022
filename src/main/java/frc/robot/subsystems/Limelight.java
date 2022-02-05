package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight extends SubsystemBase {
  /** Creates a new limelight. */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public Limelight() {}
  
  public void periodic() {}

  public enum Pipeline {
    drive, blue, red;
  }

  public void setPipeline(Pipeline pipeline) {
    switch(pipeline) {
      case drive:
        table.getEntry("pipeline").setNumber(0);
        table.getEntry("camMode").setNumber(1);
        break;
      case blue:
        table.getEntry("pipeline").setNumber(1);
        table.getEntry("camMode").setNumber(0);
        break;
      case red:
        table.getEntry("pipeline").setNumber(2);
        table.getEntry("camMode").setNumber(0);
        break;
    }

    table.getEntry("pipeline").setNumber(0);
  }

  public double getHorizontal() {
    return table.getEntry("thor").getDouble(0);
  }
  
  public double getVertical() {
    return table.getEntry("tvert").getDouble(0);
  }

  public double getTX() {
    return table.getEntry("tx").getDouble(0);
  }

  public boolean getDetected() {
    if ((table.getEntry("tv").getDouble(0)) == 1) return true;
    return false;
  }
}