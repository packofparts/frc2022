package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
  /** Creates a new limelight. */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public Limelight() {
    setPipeline(Pipeline.drive);
  }

  public enum Pipeline {
    drive, hub;
  }

  public void setPipeline(Pipeline pipeline) {
    switch(pipeline) {
      case drive:
        table.getEntry("pipeline").setNumber(0);

        // cammode 1 is drive camera
        table.getEntry("camMode").setNumber(1);
        break;
      case hub:
        table.getEntry("pipeline").setNumber(3);

        // cammode 2 is budget open cv
        table.getEntry("camMode").setNumber(0);
        break;
    }
    table.getEntry("camMode").setNumber(0);
  }

  public double getTX() {
    return table.getEntry("tx").getDouble(0);
  }

  public double getTY() {
    return table.getEntry("ty").getDouble(0);
  }

  public boolean getDetected() {
    if ((table.getEntry("tv").getDouble(0)) == 1) return true;
    return false;
  }
}