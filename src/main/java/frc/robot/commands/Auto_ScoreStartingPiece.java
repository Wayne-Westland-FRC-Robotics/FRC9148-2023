// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BendArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SlideArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_ScoreStartingPiece extends SequentialCommandGroup {
  /** Creates a new Auto_Double_Score. */
  private final BendArmSubsystem m_armBend;
  private final SlideArmSubsystem m_armSlide;
  private final ClawSubsystem m_armClaw;
  /**
   * Autonomous command for scoring the game piece that starts in the grabber.
   * @param armBend Subsystem for changing the arm angle.
   * @param armSlide Subsystem for extending/retracting the arm.
   * @param armClaw Subsystem for actuating the grabber. 
   */
  public Auto_ScoreStartingPiece(BendArmSubsystem armBend, SlideArmSubsystem armSlide, ClawSubsystem armClaw) {
    m_armBend = armBend;
    m_armSlide = armSlide;
    m_armClaw = armClaw;
    addCommands(
      new ActuateClaw(1, m_armClaw),
      new BendArmPID(m_armBend, 90.0, true),
      new SlideArm(0.1, m_armSlide).withTimeout(0.5),
      new ActuateClaw(0, m_armClaw),
      new ParallelCommandGroup(
        new SlideArm(-0.1, m_armSlide).withTimeout(0.5),
        new ActuateClaw(1, m_armClaw),
        new BendArmPID(m_armBend, 0.0, true)
      )
    );
  }
}
