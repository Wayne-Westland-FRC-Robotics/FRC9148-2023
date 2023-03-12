// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ContainerConstants;
import frc.robot.commands.ActuateClaw;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmNeutralPID;
import frc.robot.commands.Auto_ChargingDirectly;
import frc.robot.commands.Auto_ExitCommunity_Long;
import frc.robot.commands.Auto_ExitCommunity_Short;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.BendArm;
import frc.robot.commands.BrakeCommand;
//import frc.robot.commands.BendArmPID;
import frc.robot.commands.Auto_Bottom;
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
  SendableChooser<Command> m_autonomousChooser = new SendableChooser<>();

  SendableChooser<Boolean> m_controllerCount = new SendableChooser<>();
  SendableChooser<Command> m_autoType = new SendableChooser<>();

  private final Command m_tankDriveCon = new TankDrive(m_drivetrain, m_drController::getLeftY, m_drController::getRightY);
  private final Command m_arcadeDriveCon = new ArcadeDrive(m_drivetrain, m_drController::getLeftY, m_drController::getRightY);
  private final Command m_tankDriveJoy = new TankDrive(m_drivetrain, m_drJoystick1::getY, m_drJoystick2::getY);
  private final Command m_arcadeDriveJoy = new ArcadeDrive(m_drivetrain, m_drJoystick1::getY, m_drJoystick1::getX);
  
  public RobotContainer(Robot robot) {
    m_robot = robot;

    m_armBendSubsystem.setDefaultCommand(new ArmNeutralPID(m_armBendSubsystem));

    m_controllerChooser.setDefaultOption("XboxTank", m_tankDriveCon);
    m_controllerChooser.addOption("XboxArcade", m_arcadeDriveCon);
    m_controllerChooser.addOption("JoystickTank", m_tankDriveJoy);
    m_controllerChooser.addOption("JoystickArcade", m_arcadeDriveJoy);
    Shuffleboard.getTab("Select Controller").add(m_controllerChooser);

    m_autonomousChooser.setDefaultOption("Direct Charge",
      new Auto_ChargingDirectly(m_drivetrain, m_robot)
    );
    m_autonomousChooser.addOption("Bottom Auto",
      new Auto_Bottom(m_drivetrain)
    );

    m_autoType.setDefaultOption("Exit Long Side Auto", new Auto_ExitCommunity_Long(m_drivetrain));
    m_autoType.setDefaultOption("Exit Short Side Auto", new Auto_ExitCommunity_Short(m_drivetrain));

    m_controllerCount.setDefaultOption("Double", true);
    m_controllerCount.addOption("Single", false);

    //configureBindings(); [Configuring bindings in "robot" for the controller chooser!]
  }

  public void configureBindings(boolean isDouble) {
    m_drController.povLeft().whileTrue(new BalanceCommand(m_drivetrain, m_robot::getRobotTilt, m_robot::getUltra));
    m_drController.a().whileTrue(new BrakeCommand(m_drivetrain));

    if (isDouble) {
      operatorSetup(m_opController);
    } else {
      operatorSetup(m_drController);
    }
    /* 
    m_opController.a().whileTrue(new BendArm(ContainerConstants.ARM_BEND_SPEED, m_armBendSubsystem));
    m_opController.b().whileTrue(new BendArm(-ContainerConstants.ARM_BEND_SPEED, m_armBendSubsystem));

    m_opController.x().whileTrue(new ActuateClaw(0, m_clawSubsystem));
    m_opController.y().whileTrue(new ActuateClaw(1, m_clawSubsystem));

    m_opController.leftBumper().whileTrue(new SlideArm(ContainerConstants.ARM_SLIDER_SPEED, m_armSlideSubystem));
    m_opController.rightBumper().whileTrue(new SlideArm(-ContainerConstants.ARM_SLIDER_SPEED, m_armSlideSubystem));
  
    //m_opController.povLeft().whileTrue(new BendArmPID(m_armBendSubsystem, false));
    //m_opController.povRight().whileTrue(new BendArmPID(m_armBendSubsystem, true));
    */
  }

  public void operatorSetup(CommandXboxController controller) {
    controller.a().whileTrue(new BendArm(ContainerConstants.ARM_BEND_SPEED, m_armBendSubsystem));
    controller.b().whileTrue(new BendArm(-ContainerConstants.ARM_BEND_SPEED, m_armBendSubsystem));

    controller.x().whileTrue(new ActuateClaw(0, m_clawSubsystem));
    controller.y().whileTrue(new ActuateClaw(1, m_clawSubsystem));

    controller.leftBumper().whileTrue(new SlideArm(ContainerConstants.ARM_SLIDER_SPEED, m_armSlideSubystem));
    controller.rightBumper().whileTrue(new SlideArm(-ContainerConstants.ARM_SLIDER_SPEED, m_armSlideSubystem));
  }

  public Command getAutonomousCommand() {
    //return m_autoType.getSelected();
    return new Auto_ExitCommunity_Long(m_drivetrain);
  }

  public Command getControllerChooser() {
    return m_controllerChooser.getSelected();
  }
  
  public Boolean getControllerCount() {
    return m_controllerCount.getSelected();
  }

  public DrivetrainSubsystem getDrivetrain() {
    return m_drivetrain;
  }
}
