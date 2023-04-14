// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);
  //private final Solenoid asdf = new Solenoid(null, 0);
  /**
   * Subsystem for actuating the grabber.
   */
  public ClawSubsystem() {}

  public void closeClaw() {
    solenoid.set(Value.kForward);
  }
  public void openClaw() {
    solenoid.set(Value.kReverse);
    SmartDashboard.putString("compresser", "Compresser should be working!");
  }
}
