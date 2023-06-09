// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_ChargingDirectly extends SequentialCommandGroup {
  private final DrivetrainSubsystem m_drivetrain;
  private final Robot m_robot;

  /**
   * Autonomous command for driving onto the charging pad when in the middle.
   * @param drivetrain Subsystem for changing the drive wheel speed.
   * @param robot Robot instance.
   */
  public Auto_ChargingDirectly(DrivetrainSubsystem drivetrain, Robot robot) {
    m_drivetrain = drivetrain;
    m_robot = robot;
    addRequirements(m_drivetrain);

    addCommands(
      new TankDrive(m_drivetrain, ()->0.15, ()->0.15).withTimeout(AutoConstants.CHARGE_DIRECT_INITIAL_TIME),
      new Balance(m_drivetrain, m_robot).withTimeout(7)
    );
  }
}
