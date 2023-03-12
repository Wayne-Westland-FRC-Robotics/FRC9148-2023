// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SlideArmSubsystem extends SubsystemBase {
  private final CANSparkMax extendArmMotor = new CANSparkMax(ArmConstants.EXTEND_ARM_MOTOR_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder = extendArmMotor.getEncoder();

  public SlideArmSubsystem() {
    extendArmMotor.setIdleMode(IdleMode.kBrake);
  }
  
  public void slide(Double speed) {
    if ((encoder.getPosition() < 0 && speed<0) || (encoder.getPosition() > ArmConstants.ARM_EXTEND_DISTANCE_ENCODER && speed>=0)) {
      SmartDashboard.putString("CanSlide", "Arm has hit slide limit!");
      extendArmMotor.set(0);
    } else {
      SmartDashboard.putString("CanSlide", "Arm has NOT hit slide limit.");
      extendArmMotor.set(speed);
    }
  }
}
