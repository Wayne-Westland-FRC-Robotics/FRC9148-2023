// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SlideArmSubsystem;

public class SlideArm extends CommandBase {
  private final Double m_speed;
  private final SlideArmSubsystem m_armSubsystem;

  /**
   * Command for extending/retracting the arm.
   * @param speed Arm extension/retraction speed.
   * @param armSubsystem Subsystem for extending/retracting the arm.
   */
  public SlideArm(Double speed, SlideArmSubsystem armSubsystem) {
    m_speed = speed;
    m_armSubsystem = armSubsystem;
    addRequirements(m_armSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.slide(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.slide(0.0);
  }
}
