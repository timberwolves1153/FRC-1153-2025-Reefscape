// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum GamePiece {
    CORAL,
    ALGAE;
  }

  public static final Transform2d ROBOT_TRANSFORM =
      new Transform2d(
          /*x*/ Units.inchesToMeters(18),
          /*y*/ Units.inchesToMeters(0),
          /*rotation*/ Rotation2d.fromDegrees(0));
  public static final Transform2d STATION_TRANSFORM =
      new Transform2d(
          /*x*/ Units.inchesToMeters(22),
          /*y*/ Units.inchesToMeters(0),
          /*rotation*/ Rotation2d.fromDegrees(0));
  public static final Transform2d ALGAE_TRANSFORM =
      new Transform2d(
          /*x*/ Units.inchesToMeters(0),
          /*y*/ Units.inchesToMeters(0),
          /*rotation*/ Rotation2d.fromDegrees(0));
  public static final Transform2d CORAL_TRANSFORM =
      new Transform2d(
          /*x*/ Units.inchesToMeters(0),
          /*y*/ Units.inchesToMeters(-.75),
          /*rotation*/ Rotation2d.fromDegrees(0));
  public static final Transform2d AUTOALIGN_TRANSFORM =
      new Transform2d(
          /*x*/ Units.inchesToMeters(24), // 10
          /*y*/ Units.inchesToMeters(0),
          /*rotation*/ Rotation2d.fromDegrees(0));
}
