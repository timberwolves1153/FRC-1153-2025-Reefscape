// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final Translation2d fieldCenter =
      new Translation2d(fieldLength / 2, fieldWidth / 2);
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line
  public static final double algaeDiameter = Units.inchesToMeters(16);

  public static class Processor {
    public static final Pose2d centerFace =
        new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final Pose2d leftCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
  }

  public static class Reef {
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order
    public static final List<Map<ReefHeight, Pose3d>> branchPositions =
        new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise

    static {
      // Initialize faces
      centerFaces[0] =
          new Pose2d(
              Units.inchesToMeters(144.003),
              Units.inchesToMeters(158.500),
              Rotation2d.fromDegrees(180));
      centerFaces[1] =
          new Pose2d(
              Units.inchesToMeters(160.373),
              Units.inchesToMeters(186.857),
              Rotation2d.fromDegrees(120));
      centerFaces[2] =
          new Pose2d(
              Units.inchesToMeters(193.116),
              Units.inchesToMeters(186.858),
              Rotation2d.fromDegrees(60));
      centerFaces[3] =
          new Pose2d(
              Units.inchesToMeters(209.489),
              Units.inchesToMeters(158.502),
              Rotation2d.fromDegrees(0));
      centerFaces[4] =
          new Pose2d(
              Units.inchesToMeters(193.118),
              Units.inchesToMeters(130.145),
              Rotation2d.fromDegrees(-60));
      centerFaces[5] =
          new Pose2d(
              Units.inchesToMeters(160.375),
              Units.inchesToMeters(130.144),
              Rotation2d.fromDegrees(-120));

      // centerFaces[6] = centerFaces[0].rotateAround(fieldCenter, Rotation2d.k180deg);
      // centerFaces[7] = centerFaces[1].rotateAround(fieldCenter, Rotation2d.k180deg);
      // centerFaces[8] = centerFaces[2].rotateAround(fieldCenter, Rotation2d.k180deg);
      // centerFaces[9] = centerFaces[3].rotateAround(fieldCenter, Rotation2d.k180deg);
      // centerFaces[10] = centerFaces[4].rotateAround(fieldCenter, Rotation2d.k180deg);
      // centerFaces[11] = centerFaces[5].rotateAround(fieldCenter, Rotation2d.k180deg);

      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
        Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
        for (var level : ReefHeight.values()) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
          double adjustX = Units.inchesToMeters(30.738);
          double adjustY = Units.inchesToMeters(6.469);

          fillRight.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
          fillLeft.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
        }
        branchPositions.add(fillRight);
        branchPositions.add(fillLeft);
      }
    }
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final Pose2d leftIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
    public static final Pose2d middleIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
    public static final Pose2d rightIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
  }

  public enum ReefHeight {
    L4(Units.inchesToMeters(72), -90),
    L3(Units.inchesToMeters(47.625), -35),
    L2(Units.inchesToMeters(31.875), -35),
    L1(Units.inchesToMeters(18), 0);

    ReefHeight(double height, double pitch) {
      this.height = height;
      this.pitch = pitch; // in degrees
    }

    public final double height;
    public final double pitch;
  }

  public static final double aprilTagWidth = Units.inchesToMeters(6.50);
  public static final int aprilTagCount = 22;

  public static Pose2d getNearestReefFace(Pose2d currentPose) {
    return currentPose.nearest(List.of(FieldConstants.Reef.centerFaces));
  }

  public static Pose2d getNearestCoralStation(Pose2d currentPose) {
    double distanceToLeftStation =
        currentPose
            .getTranslation()
            .getDistance(FieldConstants.CoralStation.leftCenterFace.getTranslation());
    double distanceToRightStation =
        currentPose
            .getTranslation()
            .getDistance(FieldConstants.CoralStation.rightCenterFace.getTranslation());

    SmartDashboard.putNumber("Distance to Left Station", distanceToLeftStation);
    SmartDashboard.putNumber("Distance to Right Station", distanceToRightStation);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    if (isFlipped) {
      if (distanceToLeftStation > distanceToRightStation) {
        return FieldConstants.CoralStation.leftCenterFace;
      } else {
        return FieldConstants.CoralStation.rightCenterFace;
      }
    } else {
      if (distanceToLeftStation > distanceToRightStation) {
        return FieldConstants.CoralStation.rightCenterFace;
      } else {
        return FieldConstants.CoralStation.leftCenterFace;
      }
    }
  }

  public static double getNearestCage(Pose2d currentPose) {

    Pose2d farCage;
    Pose2d middleCage;
    Pose2d closeCage;

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    if (isFlipped) {
      farCage =
          new Pose2d(FieldConstants.Barge.farCage, new Rotation2d())
              .rotateAround(fieldCenter, Rotation2d.k180deg);
      middleCage =
          new Pose2d(FieldConstants.Barge.middleCage, new Rotation2d())
              .rotateAround(fieldCenter, Rotation2d.k180deg);

      closeCage =
          new Pose2d(FieldConstants.Barge.closeCage, new Rotation2d())
              .rotateAround(fieldCenter, Rotation2d.k180deg);
      ;
    } else {
      farCage = new Pose2d(FieldConstants.Barge.farCage, new Rotation2d());
      middleCage = new Pose2d(FieldConstants.Barge.middleCage, new Rotation2d());
      closeCage = new Pose2d(FieldConstants.Barge.closeCage, new Rotation2d());
    }

    double closestDistance;

    double distanceToFarCage = currentPose.getTranslation().getDistance(farCage.getTranslation());

    double distanceToMiddleCage =
        currentPose.getTranslation().getDistance(middleCage.getTranslation());

    double distanceToCloseCage =
        currentPose.getTranslation().getDistance(closeCage.getTranslation());

    SmartDashboard.putNumber("Distance to Far Cage", distanceToFarCage);
    SmartDashboard.putNumber("Distance to Middle Cage", distanceToMiddleCage);
    SmartDashboard.putNumber("Distance to Close Cage", distanceToCloseCage);

    if (distanceToFarCage < distanceToMiddleCage && distanceToFarCage < distanceToCloseCage) {
      closestDistance = distanceToFarCage;
      SmartDashboard.putNumber("distance from cage", closestDistance);
      return closestDistance;

    } else if (distanceToCloseCage < distanceToMiddleCage
        && distanceToCloseCage < distanceToFarCage) {
      closestDistance = distanceToCloseCage;
      SmartDashboard.putNumber("distance from cage", closestDistance);
      return closestDistance;

    } else if (distanceToMiddleCage < distanceToFarCage
        && distanceToMiddleCage < distanceToCloseCage) {
      closestDistance = distanceToMiddleCage;
      SmartDashboard.putNumber("distance from cage", closestDistance);
      return closestDistance;

    } else {
      closestDistance = distanceToMiddleCage;
      SmartDashboard.putNumber("distance from cage", closestDistance);
      return closestDistance;
    }
  }
}
