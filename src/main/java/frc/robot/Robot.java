// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.UltrasonicConstants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  //Accelerometer (Gyro)
  private BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  // Ultrasonics:
  public AnalogPotentiometer m_ultrasonicFL = new AnalogPotentiometer(UltrasonicConstants.ULTRASONIC_FRONT_LEFT);
  public AnalogPotentiometer m_ultrasonicFR = new AnalogPotentiometer(UltrasonicConstants.ULTRASONIC_FRONT_RIGHT);
  public AnalogPotentiometer m_ultrasonicBL = new AnalogPotentiometer(UltrasonicConstants.ULTRASONIC_BACK_LEFT);
  public AnalogPotentiometer m_ultrasonicBR = new AnalogPotentiometer(UltrasonicConstants.ULTRASONIC_BACK_RIGHT);

  public Double getRobotTilt() {
    return Math.asin(accelerometer.getY()*180/Math.PI);
  }
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer(this);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.getDrivetrain().setDefaultCommand(
      m_robotContainer.getControllerChooser()
    );
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("FL_ULTRA", m_ultrasonicFL.get());
    SmartDashboard.putNumber("FR_ULTRA", m_ultrasonicFR.get());
    SmartDashboard.putNumber("BL_ULTRA", m_ultrasonicBL.get());
    SmartDashboard.putNumber("BR_ULTRA", m_ultrasonicBR.get());
  }

  public boolean getUltra() {
    return 
      m_ultrasonicFR.get() >= UltrasonicConstants.ULTRASONIC_LIMIT ||
      m_ultrasonicFL.get() >= UltrasonicConstants.ULTRASONIC_LIMIT ||
      m_ultrasonicBR.get() >= UltrasonicConstants.ULTRASONIC_LIMIT ||
      m_ultrasonicBL.get() >= UltrasonicConstants.ULTRASONIC_LIMIT ;
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
