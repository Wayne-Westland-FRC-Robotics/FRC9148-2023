// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ContainerConstants;
import frc.robot.commands.ActuateClaw;
import frc.robot.commands.BendArm;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.SlideArm;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.MoveArmSubsystem;
import frc.robot.subsystems.SlideArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final Robot m_robot;
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  private final MoveArmSubsystem m_armBendSubsystem = new MoveArmSubsystem();
  private final SlideArmSubsystem m_armSlideSubystem = new SlideArmSubsystem();
  private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();

  private final CommandXboxController m_drController = new CommandXboxController(ContainerConstants.OPERATOR_CONTROLLER_PORT);
  private final CommandXboxController m_opController = new CommandXboxController(ContainerConstants.OPERATOR_CONTROLLER_PORT);
  
  public RobotContainer(Robot robot) {
    m_robot = robot;
    m_drivetrain.setDefaultCommand(
      new DefaultDrive(m_drivetrain, m_drController::getLeftX, m_drController::getRightY, m_robot::getUltra, this::goingForCharge)
    );
    configureBindings();
  }

  private void configureBindings() {
    m_opController.a().whileTrue(new BendArm(ContainerConstants.ARM_BEND_SPEED, m_armBendSubsystem));
    m_opController.b().whileTrue(new BendArm(-ContainerConstants.ARM_BEND_SPEED, m_armBendSubsystem));

    m_opController.x().whileTrue(new ActuateClaw(0, m_clawSubsystem));
    m_opController.y().whileTrue(new ActuateClaw(1, m_clawSubsystem));

    m_opController.leftBumper().whileTrue(new SlideArm(ContainerConstants.ARM_SLIDER_SPEED, m_armSlideSubystem));
    m_opController.rightBumper().whileTrue(new SlideArm(-ContainerConstants.ARM_SLIDER_SPEED, m_armSlideSubystem));
  }

  public boolean goingForCharge() {
    return m_opController.povDown().getAsBoolean();
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
