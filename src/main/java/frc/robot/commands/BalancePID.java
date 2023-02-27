// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
/* 
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.ControlSystemConstants;
*/
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalancePID extends CommandBase {
  private final DrivetrainSubsystem m_drivetrain;

  // NEEDS TO BE FINISHED!
  /*
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    ControlSystemConstants.kSBal,
    ControlSystemConstants.kVBal,
    ControlSystemConstants.kABal
  );
  
  private final PIDController pidController1 = new PIDController(
    ControlSystemConstants.kPBal,
    0,
    ControlSystemConstants.kDBal
  );
  */

  public BalancePID(DrivetrainSubsystem drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}
