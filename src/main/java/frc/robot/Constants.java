// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13;


  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;


  }
  public static class ClimberConstants {
    public static final int CAN_ID_CLIMBER_MOTOR_1 = 9;
    public static final int CAN_ID_CLIMBER_MOTOR_2 = 10;

    public static final double CLIMBER_SPEED = 0.1;

    public static final double ROTATION_DISTANCE = 60;

    public static final double CLIMBER_P = 0.05;
    public static final double CLIMBER_I = 0.0000001;
    public static final double CLIMBER_D = 0.000006;
    public static final double CLIMBER_IZONE = 0.0;
    public static final double CLIMBER_FF = 0.000015;
    public static final double CLIMBER_MAX = 0.5;
    public static final double CLIMBER_MIN = -0.5;

  }

  public static class ArmConstants {
    public static final int CAN_ID_ARM_MOTOR_L = 11;
    public static final int CAN_ID_ARM_MOTOR_R = 12;

    public static final double ARM_SPEED = 0.6;

    public static final double ARM_P = 0;
    public static final double ARM_I = 0;
    public static final double ARM_D = 0;
    public static final double ARM_IZONE = 0.0;
    public static final double ARM_FF = 0.000015;
    public static final double ARM_MAX = 0.5;
    public static final double ARM_MIN = -0.5;

    public static final double conversionFactor = 0.005;
  }

  public static class IntakeConstants{
    public static final int CAN_ID_FlyWheel_MOTOR_T = 13;
    public static final int CAN_ID_FlyWheel_MOTOR_B = 14;
    public static final double FlyWheel_SPEED = 0.5;
  }
}
