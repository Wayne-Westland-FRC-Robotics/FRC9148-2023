// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ContainerConstants {
    public final static int DRIVER_CONTROLLER_PORT = 0;
    public final static int OPERATOR_CONTROLLER_PORT = 1;

    public final static int DRIVER_JOYSTICK_PORT_1 = 0;
    public final static int DRIVER_JOYSTICK_PORT_2 = 1;
    public final static int OPERATOR_JOYSTICK_PORT = 2;

    public final static double ARM_BEND_SPEED = 0.25;
    public final static double ARM_SLIDER_SPEED = 0.1;
  }

  public static class DrivetrainConstants {
    public final static int LEFT_MOTOR_ID_1 = 1;
    public final static int LEFT_MOTOR_ID_2 = 2;
    public final static int RIGHT_MOTOR_ID_1 = 3;
    public final static int RIGHT_MOTOR_ID_2 = 4;
  }

  public static class ArmConstants {
    public final static int BEND_ARM_MOTOR_ID = 5;
    public final static int EXTEND_ARM_MOTOR_ID = 6;

    public final static double ARM_BEND_RADIUS_ENCODER = 42*70/3;
    public final static double ARM_EXTEND_DISTANCE_ENCODER = 42*4*3;

    public final static int CLAW_FORWARD = 1;
    public final static int CLAW_REVERSE = 0;
  }

  public static class UltrasonicConstants {
    public final static int ULTRASONIC_FRONT_LEFT = 0;
    public final static int ULTRASONIC_FRONT_RIGHT = 1;
    public final static int ULTRASONIC_BACK_LEFT = 2;
    public final static int ULTRASONIC_BACK_RIGHT = 3;

    public final static double ULTRASONIC_LIMIT = 10;
  }

  public static class ControlSystemConstants {
    public final static double kSArm = 0.071297;
    public final static double kVArm = 0.0001291;
    public final static double kAArm = 0.019404;

    public final static double kPArm = 0.000366;
    public final static double kDArm = 0;
    // pid balance
    public final static double kSBal = 0;
    public final static double kVBal = 0;
    public final static double kABal = 0;

    public final static double kPBal = 0;
    public final static double kDBal = 0;

    public final static double ARM_UPPER_POSITION = 0;
    public final static double ARM_LOWER_POSITION = 120;
    // non-pid balance
    public final static double BALANCE_TILT_LIMIT = 10; // This is in degrees.
    public final static double BALANCE_SPEED = 0.07; // Speed of the robot when trying to balance
  }

  public static class AutoConstants {
    public final static double CHARGE_DIRECT_INITIAL_TIME = 2; // This is in seconds
  }
}
