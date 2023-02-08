// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControlSystemConstants;

public class BendArmPID extends CommandBase {
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    ControlSystemConstants.kS,
    ControlSystemConstants.kV,
    ControlSystemConstants.kA
  );
  
  
  
  private final PIDController pidController1 = new PIDController(
    ControlSystemConstants.kP,
    0,
    ControlSystemConstants.kD
  );
  public BendArmPID() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
 
}
