// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.BendArmSubsystem;

public class BendToAngle extends CommandBase {
  private final BendArmSubsystem m_armBend;
  private double m_goalAngle;
  private RelativeEncoder encoder;
  public BendToAngle(BendArmSubsystem armBend, double goalAngle) {
    m_armBend = armBend;
    m_goalAngle = goalAngle;
    addRequirements(m_armBend);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoder = m_armBend.getEncoder();

    if (m_goalAngle > ArmConstants.ARM_BEND_UPPER_LIMIT) m_goalAngle = ArmConstants.ARM_BEND_UPPER_LIMIT;
    else if (m_goalAngle < ArmConstants.ARM_BEND_LOWER_LIMIT) m_goalAngle = ArmConstants.ARM_BEND_LOWER_LIMIT;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (encoder.getPosition() <= m_goalAngle - ArmConstants.AUTO_ARM_BEND_DEADZONE) {
      m_armBend.bend(0.1);
    } else if (encoder.getPosition() >= m_goalAngle + ArmConstants.AUTO_ARM_BEND_DEADZONE) {
      m_armBend.bend(-0.1);
    } else {
      end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
