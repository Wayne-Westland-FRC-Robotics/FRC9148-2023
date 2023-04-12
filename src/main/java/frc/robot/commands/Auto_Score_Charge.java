// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.BendArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.SlideArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Score_Charge extends SequentialCommandGroup {
  private final BendArmSubsystem m_armBend;
  private final SlideArmSubsystem m_armSlide;
  private final ClawSubsystem m_armClaw;
  private final DrivetrainSubsystem m_drive;
  private final Robot m_robot;
  public Auto_Score_Charge(BendArmSubsystem armBend, SlideArmSubsystem armSlide, ClawSubsystem armClaw, DrivetrainSubsystem drive, Robot robot) {
    m_armBend = armBend;
    m_armSlide = armSlide;
    m_armClaw = armClaw;
    m_drive = drive;
    m_robot = robot;
    addCommands(
      new ActuateClaw(1, m_armClaw),
      new SlideArm(-0.1, m_armSlide).withTimeout(0.5),
      new BendArmPID(m_armBend, 90d, true).withTimeout(3),
      new ActuateClaw(0, m_armClaw),
      new TankDrive(m_drive, () -> 0.3, () -> 0.3).withTimeout(3),
      new BalanceNoUltra(m_drive, m_robot)
    );
  }
}
