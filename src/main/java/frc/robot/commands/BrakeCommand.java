// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BrakeCommand extends CommandBase {
  private final DrivetrainSubsystem m_drivetrain;
  /**
   * Enables brakes for the driving motors when running
   * @param drivetrain Subsystem for changing the drive wheel speed.
   */
  public BrakeCommand(DrivetrainSubsystem drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.startBrake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopBrake();
  }
}
