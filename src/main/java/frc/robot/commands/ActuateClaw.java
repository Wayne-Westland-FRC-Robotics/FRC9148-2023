// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ActuateClaw extends CommandBase {
  private final double m_actuationType;
  private final ClawSubsystem m_clawSubsystem;
  public ActuateClaw(double actuationType, ClawSubsystem clawSubsystem) {
    m_actuationType = actuationType;
    m_clawSubsystem = clawSubsystem;
    addRequirements(m_clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_actuationType == 0) {
      m_clawSubsystem.openClaw();
    } else {
      m_clawSubsystem.closeClaw();
    }
  }

}
