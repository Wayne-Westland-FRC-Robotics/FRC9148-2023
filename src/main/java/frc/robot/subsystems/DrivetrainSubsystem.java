// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(
    new CANSparkMax(DrivetrainConstants.LEFT_MOTOR_ID_1, MotorType.kBrushless),
    new CANSparkMax(DrivetrainConstants.LEFT_MOTOR_ID_2, MotorType.kBrushless)
  );
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(
    new CANSparkMax(DrivetrainConstants.RIGHT_MOTOR_ID_1, MotorType.kBrushless),
    new CANSparkMax(DrivetrainConstants.RIGHT_MOTOR_ID_2, MotorType.kBrushless)
  );

  public DrivetrainSubsystem() {}

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  public void drive(Double leftSpeed, Double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }
}
