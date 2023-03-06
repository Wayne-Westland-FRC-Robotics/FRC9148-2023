// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ContainerConstants;
import frc.robot.commands.ActuateClaw;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.BendArm;
import frc.robot.commands.BendArmPID;
import frc.robot.commands.TankDrive;
import frc.robot.commands.SlideArm;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.BendArmSubsystem;
import frc.robot.subsystems.SlideArmSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final Robot m_robot;
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  private final BendArmSubsystem m_armBendSubsystem = new BendArmSubsystem();
  private final SlideArmSubsystem m_armSlideSubystem = new SlideArmSubsystem();
  private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();

  private final CommandXboxController m_drController = new CommandXboxController(ContainerConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController m_opController = new CommandXboxController(ContainerConstants.OPERATOR_CONTROLLER_PORT);
  private final CommandJoystick m_drJoystick1= new CommandJoystick(ContainerConstants.DRIVER_JOYSTICK_PORT_1);
  private final CommandJoystick m_drJoystick2 = new CommandJoystick(ContainerConstants.DRIVER_JOYSTICK_PORT_2);

  SendableChooser<Command> m_controllerChooser = new SendableChooser<>();
  private final Command m_tankDriveCon = new TankDrive(m_drivetrain, m_drController::getLeftY, m_drController::getRightY);
  private final Command m_arcadeDriveCon = new ArcadeDrive(m_drivetrain, m_drController::getLeftY, m_drController::getRightY);
  private final Command m_tankDriveJoy = new TankDrive(m_drivetrain, m_drJoystick1::getY, m_drJoystick2::getY);
  private final Command m_arcadeDriveJoy = new ArcadeDrive(m_drivetrain, m_drJoystick1::getY, m_drJoystick1::getX);

  public RobotContainer(Robot robot) {
    m_robot = robot;

    m_controllerChooser.setDefaultOption("XboxTank", m_tankDriveCon);
    m_controllerChooser.addOption("XboxArcade", m_arcadeDriveCon);
    m_controllerChooser.addOption("JoystickTank", m_tankDriveJoy);
    m_controllerChooser.addOption("JoystickArcade", m_arcadeDriveJoy);
    Shuffleboard.getTab("Select Controller").add(m_controllerChooser);

    configureBindings();
  }

  private void configureBindings() {
    m_drController.povDown().whileTrue(new BalanceCommand(m_drivetrain, m_robot::getRobotTilt, m_robot::getUltra));

    m_opController.a().whileTrue(new BendArm(ContainerConstants.ARM_BEND_SPEED, m_armBendSubsystem));
    m_opController.b().whileTrue(new BendArm(-ContainerConstants.ARM_BEND_SPEED, m_armBendSubsystem));

    m_opController.x().whileTrue(new ActuateClaw(0, m_clawSubsystem));
    m_opController.y().whileTrue(new ActuateClaw(1, m_clawSubsystem));

    m_opController.leftBumper().whileTrue(new SlideArm(ContainerConstants.ARM_SLIDER_SPEED, m_armSlideSubystem));
    m_opController.rightBumper().whileTrue(new SlideArm(-ContainerConstants.ARM_SLIDER_SPEED, m_armSlideSubystem));
  
    m_opController.povLeft().whileTrue(new BendArmPID(m_armBendSubsystem, false));
    m_opController.povRight().whileTrue(new BendArmPID(m_armBendSubsystem, true));
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public Command getControllerChooser() {
    return m_controllerChooser.getSelected();
  }

  public DrivetrainSubsystem getDrivetrain() {
    return m_drivetrain;
  }
}
