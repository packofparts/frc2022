// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.XboxController;

// public class Shooter extends SubsystemBase {
//   /** Creates a new Shooter. */
//   XboxController xbox = new XboxController(Constants.xboxPort);
//   Joystick _joystick = new Joystick(Constants.joystickPort);
//   TalonFX flyWheel = new TalonFX(Constants.flyWheelPort);
//   TalonFX nonFlyWheel = new TalonFX(0);
//   double RPM=0.0;
//   boolean isXbox = true;
  
//   public Shooter() {
//     if(isXbox){
//       if (xbox.getPOV() == 0) {
//         if (RPM+Constants.increment<=7500) {
//           setRPM(RPM+Constants.increment);
//           setRPMNoFLy(RPM+Constants.increment);
//         }
//       }
//       if (xbox.getPOV() == 180) {
//         if (RPM-Constants.increment>=0) {
//           setRPM(RPM-Constants.increment);
//           setRPMNoFLy(RPM-Constants.increment);
//         }
//       }
//     }

//     else {

//       double stick = _joystick.getRawAxis(2);
//       flyWheel.set(ControlMode.Velocity, -stick);
//       nonFlyWheel.set(ControlMode.Velocity,-stick);
//     }
//   }

//   public void setRPM(double RPM) {
//     flyWheel.set(ControlMode.Velocity, RPM);
//   }
//   public void setRPMNoFLy(double RPM) {
//     nonFlyWheel.set(ControlMode.Velocity, RPM);
//   }
//   @Override
//   public void periodic() {   
//   }
// }
