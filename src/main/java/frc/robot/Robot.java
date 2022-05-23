// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.schedulers.SequentialScheduler;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autoPath.FourBallComplex;
import frc.robot.autoPath.FourBallSimple;
import frc.robot.autoPath.OneBallSimple;
import frc.robot.autoPath.ThreeBallAuto;
import frc.robot.autoPath.TrajectoryGeneration;
import frc.robot.autoPath.TwoBallComplex;
import frc.robot.autoPath.TwoBallSimple;
import frc.robot.commands.MoveBy;
import frc.robot.commands.PlayMusic;
import frc.robot.commands.TurnBy;
import frc.robot.subsystems.Limelight.Pipeline;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static RobotContainer m_robotContainer;
  // private AddressableLED m_led = new AddressableLED(11);

  // Reuse buffer
  // Default to a length of 60, start empty output
  // private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(100);
  
  private SendableChooser<Pipeline> ballColor = new SendableChooser<>();
  private SendableChooser<Command> autoCommand = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    ballColor.setDefaultOption("blue", Pipeline.blue);
    ballColor.addOption("red", Pipeline.red);
    SmartDashboard.putData("Ball Color", ballColor);

    autoCommand.setDefaultOption("Two Ball (Complex)", new TwoBallComplex(m_robotContainer.drive, m_robotContainer.tube, m_robotContainer.shooter, m_robotContainer.limelight));
    autoCommand.addOption("Two Ball (Simple)", new TwoBallSimple(m_robotContainer.drive, m_robotContainer.tube, m_robotContainer.shooter));
    autoCommand.addOption("Four Ball (Simple)", new FourBallSimple(m_robotContainer.drive, m_robotContainer.tube, m_robotContainer.shooter, m_robotContainer.limelight));
    autoCommand.addOption("Four Ball (Complex)", new FourBallComplex(m_robotContainer.drive, m_robotContainer.tube, m_robotContainer.shooter, m_robotContainer.limelight));
    autoCommand.addOption("One Ball (Simple)", new OneBallSimple(m_robotContainer.drive, m_robotContainer.tube, m_robotContainer.shooter));
    autoCommand.addOption("moveBy 5ft", new MoveBy(m_robotContainer.drive, 5));
    autoCommand.addOption("turnBy 180", new TurnBy(m_robotContainer.drive, 180));
    autoCommand.addOption("Three Ball (Auto)", new ThreeBallAuto(m_robotContainer.drive, m_robotContainer.tube, m_robotContainer.shooter, m_robotContainer.limelight));
    autoCommand.addOption("SUsYeet", new TrajectoryGeneration(m_robotContainer.drive));
    // LED code
    /**
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 55, 100, 100);
   } */
  }

  public Pipeline getBallColor() {
    return ballColor.getSelected();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

   
  //  m_led.setData(m_ledBuffer);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.drive.stop();
    m_robotContainer.shooter.stopShooter();
    m_robotContainer.tube.stopTube();
    m_robotContainer.tube.setPneumatics(false);
    //m_robotContainer.climb.setPneumatics(false);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //dont take joystick inputs in auto
    m_autonomousCommand = autoCommand.getSelected();

    CommandScheduler.getInstance().cancelAll();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null && !m_autonomousCommand.isScheduled()) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // if (!m_autonomousCommand.isFinished() && !m_autonomousCommand.isScheduled()) {
    //   m_autonomousCommand.schedule();
    // }
  }

  @Override
  public void teleopInit() {

    System.out.println("Gayness");
    //enable joystick inputs in teleop
    m_robotContainer.drive.setShouldDrive(true);
    // m_robotContainer.limelight.setPipeline(Pipeline.drive);


    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    SmartDashboard.putBoolean("gyroReset", false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(m_robotContainer.joysticks.getMusicButton()) {
      CommandScheduler.getInstance().schedule(new PlayMusic());
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
