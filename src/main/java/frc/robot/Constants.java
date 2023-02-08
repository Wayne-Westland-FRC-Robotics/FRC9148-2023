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
    public final static double kS = 0.071297;
    public final static double kV = 0.0001291;
    public final static double kA = 0.019404;

    public final static double kP = 0.000366;
    public final static double kD = 0;
  }
}
