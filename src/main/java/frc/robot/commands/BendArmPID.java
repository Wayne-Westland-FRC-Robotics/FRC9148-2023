// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControlSystemConstants;
import frc.robot.subsystems.BendArmSubsystem;

public class BendArmPID extends CommandBase {
  private final BendArmSubsystem m_moveArmSubsystem;
  private final Boolean m_goTop;

  /* 
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    ControlSystemConstants.kS,
    ControlSystemConstants.kV,
    ControlSystemConstants.kA
  );
  */
  
  private final PIDController pidController1 = new PIDController(
    ControlSystemConstants.kPArm,
    0,
    ControlSystemConstants.kDArm
  );
  public BendArmPID(BendArmSubsystem moveArmSubsystem, Boolean goTop) {
    m_moveArmSubsystem = moveArmSubsystem;
    m_goTop = goTop;
    addRequirements(m_moveArmSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_moveArmSubsystem.bend(calc(m_moveArmSubsystem.getEncoder(), m_goTop));
    /* 
    SmartDashboard.putNumber("Feedforward calculation", feedforward.calculate(
      speed
    ));
    */
  }

  @Override
  public void initialize() {
    m_moveArmSubsystem.startBrake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_moveArmSubsystem.bend(0.0);
  }
  
  private double calc(RelativeEncoder encoder, Boolean goTop) {
    return pidController1.calculate(
      encoder.getPosition(), (goTop ? ControlSystemConstants.ARM_UPPER_POSITION : ControlSystemConstants.ARM_LOWER_POSITION)
    );// + feedforward.calculate(speed);
  }

  @Override
  public boolean isFinished() {
    return pidController1.atSetpoint();
  }
  // Returns true when the command should end.
 
}
