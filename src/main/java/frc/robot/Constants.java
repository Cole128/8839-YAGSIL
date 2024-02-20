// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

  public static final class Auton {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.0020645, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.01, 0, 0);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final int ControllerPort = 0;
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
    public static final double Z_DEADBAND = 0.01;
    public static final double RIGHT_X_DEADBAND = 0.01;
    public static final double TURN_CONSTANT = 0.75;
  }

  public static class MotorConstants {
    public static final int MOTOR_INTAKE_ID = 13;
    public static final int MOTOR_INTAKE_ANGLE_ID = 14;
    public static final int MOTOR_ELEVATOR_LEFT_ID = 15;
    public static final int MOTOR_ELEVATOR_RIGHT_ID = 16;
    public static final int MOTOR_FLYWHEEL_LEFT_ID = 18;
    public static final int MOTOR_FLYWHEEL_RIGHT_ID = 19;
    public static final int MOTOR_FEEDER_ID = 20;
    
  }

  public static class Buttons {
    public static final int RETRACT_INTAKE_JOINT_BUTTON_ID = 1;
    public static final int EXTEND_INTAKE_JOINT_BUTTON_ID = 2;
    public static final int INTAKE_BUTTON_ID = 3;
    public static final int SHOOTER_BUTTON_ID = 4;
    public static final int ELEVATOR_EXTEND_BUTTON_ID = 5;
    public static final int ELEVATOR_RETRACT_BUTTON_ID = 6;

  }

  public static class Intake {
    public static final double kOutP = 0.06;
    public static final double kInP = 0.06;
    public static final double ki = 0.0;
    public static final double kd = 0.0;
    public static final int INTAKE_CURRENT_LIMIT = 25;
  }

  public static class JointConstants {
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kIz = 0.0;
    public static final double kMaxOutput = 0.7;
    public static final double kMinOutput = -0.7;

    public static final double kUpperLimit = 1.0;
    public static final double kLowerLimit = 0.0;

  }
}