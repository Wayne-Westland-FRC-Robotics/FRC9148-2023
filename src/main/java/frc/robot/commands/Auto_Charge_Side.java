// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Charge_Side extends SequentialCommandGroup {
  private final DrivetrainSubsystem m_drive;
  private final Robot m_robot;
  private final int m_side; // 0: Left, 1: Right
  /** Creates a new Auto_Charge_Side. */
  public Auto_Charge_Side(DrivetrainSubsystem drive, Robot robot, int side) {
    m_drive = drive;
    m_robot = robot;
    m_side = side;
    addCommands(
      new TankDrive(m_drive, ()->(m_side==0 ? 0.5 : 0.2), ()->(m_side==0 ? 0.2 : 0.5)).withTimeout(1),
      new TankDrive(m_drive, ()->0.2, ()->0.2).withTimeout(1),
      new TankDrive(m_drive, ()->(m_side==0 ? 0.2 : 0.5), ()->(m_side==0 ? 0.5 : 0.2)),withTimeout(1),
      new TankDrive(m_drive, ()->0.2, ()->0.2).withTimeout(1),
      new BalanceNoUltra(m_drive, m_robot).withTimeout(4)
    );
  }
}
