package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Counter;

public class Lidar extends SubsystemBase {
  private Counter counter;
  private static final double offset = 0.0;
  //this value will be determined with testing
  private static final double factor = 100000; 
  // this is for centimeters inches will be 39400 but lots of testing is required

  
  public Lidar() { 
    counter = new Counter(0);
    counter.setMaxPeriod(1.0);
    counter.setSemiPeriodMode(true);
    counter.reset();
  }
  @Override
  public void periodic() {
  }
  public double getDistance() {
    if (counter.get() < 1) {
        return 0.0;
    }
    return (counter.getPeriod() * factor + offset); // Use lin reg to obtain values
}
}
