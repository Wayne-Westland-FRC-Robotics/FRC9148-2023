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
  private final Double m_angle; // Degrees
  private final Boolean m_endAtSetpoint;

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
  /**
   * Command for changing the angle of the arm..
   * @param moveArmSubsystem Subsystem for changing the arm angle.
   * @param angle Specifies the desired angle to be bent to.
   * @param endAtSetpoint Specifies whether or not the arm should stop when it reaches
   * the desired angle.
   */
  public BendArmPID(BendArmSubsystem moveArmSubsystem, Double angle, Boolean endAtSetpoint) {
    m_moveArmSubsystem = moveArmSubsystem;
    m_angle = angle;
    m_endAtSetpoint = endAtSetpoint;
    addRequirements(m_moveArmSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_moveArmSubsystem.bend(calc(m_moveArmSubsystem.getEncoder(), m_angle));
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
  
  private double calc(RelativeEncoder encoder, Double angle) {
    return pidController1.calculate(
      encoder.getPosition(), angle
    );// + feedforward.calculate(speed);
  }

  @Override
  public boolean isFinished() {
    if (m_endAtSetpoint) {
      return pidController1.atSetpoint();
    }
    return false;
  }
  // Returns true when the command should end.
 
}
