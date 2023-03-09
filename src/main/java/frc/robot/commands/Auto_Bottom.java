// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Bottom extends SequentialCommandGroup {
  private final DrivetrainSubsystem m_drivetrain;

  public Auto_Bottom(DrivetrainSubsystem drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);

    addCommands(
      new TankDrive(m_drivetrain, ()->0.05, ()->0.05).withTimeout(3)
    );
  }
}
