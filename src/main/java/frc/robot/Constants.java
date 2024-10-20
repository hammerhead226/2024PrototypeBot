// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static enum Mode {
      REAL,
      SIM,
      REPLAY

  }

  public static Mode getMode() {
    return switch (currentMode) {
      case REAL -> Mode.REAL;
      case SIM -> Mode.SIM;
      case REPLAY -> Mode.REPLAY;
    };
  }

  public static final Mode currentMode = Mode.REAL;
  public static final String CANBUS = "CAN Bus 1";
  public static final String Limelight = "Limelight 1";

  public static class SwerveConstants {
    public static final double MAX_LINEAR_SPEED = 5.56;
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(26.0);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(26.0);
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = 0.45 * MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
    public static final double OPEN_LOOP_RAMP_SEC = 0.05;
  }

  public static class ModuleConstants {
    public static final double WHEEL_RADIUS = Units.inchesToMeters(3.9 / 2.);

    public static final double DRIVE_GEAR_RATIO = 6.12;
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    public static final double DRIVE_STATOR_CURRENT_LIMIT = 75.0;
    public static final boolean DRIVE_STATOR_CURRENT_LIMIT_ENABLED = true;
    public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 42.0;
    public static final boolean DRIVE_SUPPLY_CURRENT_LIMIT_ENABLED = true;

    public static final double TURN_STATOR_CURRENT_LIMIT = 30.0;
    public static final boolean TURN_STATOR_CURRENT_LIMIT_ENABLED = true;
    public static final double TURN_SUPPLY_CURRENT_LIMIT = 30.0;
    public static final boolean TURN_SUPPLY_CURRENT_LIMIT_ENABLED = true;
  }
}
