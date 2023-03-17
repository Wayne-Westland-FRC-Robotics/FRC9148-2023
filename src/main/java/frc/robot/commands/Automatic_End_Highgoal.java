// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.BendArmSubsystem;
import frc.robot.subsystems.SlideArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Automatic_End_Highgoal extends SequentialCommandGroup {
  private final BendArmSubsystem m_armBend;
  private final SlideArmSubsystem m_slideArm;

  public Automatic_End_Highgoal(BendArmSubsystem armBend, SlideArmSubsystem slideArm) {
    m_armBend = armBend;
    m_slideArm = slideArm;
    addCommands(
      new BendToAngle(m_armBend, ArmConstants.ARM_BEND_UPPER_LIMIT),
      new SlideArm(0.1, m_slideArm).withTimeout(0.5)
    );
  }
}
