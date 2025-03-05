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

package frc.robot.subsystems.alignment;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class AlignmentConstants {
  // Camera names, must match names configured on coprocessor
  public static String cameraName = "alignmentCam"; // coral scoring side

  public static double REEF_TARGET_HEIGHT = Units.inchesToMeters(12);

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera =
      new Transform3d(
          Units.inchesToMeters(6.75),
          Units.inchesToMeters(-10.75 + 1.5),
          Units.inchesToMeters(12),
          new Rotation3d(0.0, Units.degreesToRadians(7.5), Units.degreesToRadians(25)));

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  public static List<Integer> REEF_TAGS =
      List.of(
          6, 7, 8, 9, 10, 11, // red
          17, 18, 19, 20, 21, 22); // blue
}
