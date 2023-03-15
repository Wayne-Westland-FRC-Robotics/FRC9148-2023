// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Constants.ControlSystemConstants;
import frc.robot.subsystems.BendArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SlideArmSubsystem;

public class AutomaticScore extends CommandBase {
  private final BendArmSubsystem m_armBend;
  private final SlideArmSubsystem m_armSlide;
  private final ClawSubsystem m_claw;
/* 
  private final PIDController pidController1 = new PIDController(
    ControlSystemConstants.kPArm,
    0,
    ControlSystemConstants.kDArm
  );
*/
  public AutomaticScore(BendArmSubsystem armBend, SlideArmSubsystem armSlide, ClawSubsystem claw) {
    m_armBend = armBend;
    m_armSlide = armSlide;
    m_claw = claw;

    addRequirements(m_armBend, m_armSlide, m_claw);
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_armBend.bend(calc(m_armBend.getEncoder(), null)); // Setup gotop
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
/* 
  private double calc(RelativeEncoder encoder, Boolean goTop) {
    return pidController1.calculate(
      encoder.getPosition(), (goTop ? ControlSystemConstants.ARM_UPPER_POSITION : ControlSystemConstants.ARM_LOWER_POSITION)
    );// + feedforward.calculate(speed);
  }
*/
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
