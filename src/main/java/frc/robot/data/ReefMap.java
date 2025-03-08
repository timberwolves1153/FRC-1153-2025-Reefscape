package frc.robot.data;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive.TargetReefFace;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class ReefMap {

  private Map<DesiredReefPosition, Pose2d> map;
  private List<Pose2d> redReefFaces;
  private List<Pose2d> blueReefFaces;
  private Map<Pose2d, Integer> reefFacePoseToAprilTagIdMap;
  private Map<Pose2d, TargetReefFace> reefFacePoseToTargetReefFace;

  public ReefMap() {
    this.map = buildReefMap();
    this.redReefFaces = buildReefFaceList(true);
    this.blueReefFaces = buildReefFaceList(false);
    this.reefFacePoseToAprilTagIdMap = buildReefFacePoseToAprilTagMap();
    this.reefFacePoseToTargetReefFace = buildReefFacePoseToTarget();
  }

  public Map<DesiredReefPosition, Pose2d> getReefMap() {
    return this.map;
  }

  public Pose2d getClosestReefFacePose(boolean isRedAlliance, Pose2d currentPose) {
    if (isRedAlliance) {
      return currentPose.nearest(redReefFaces);
    } else {
      return currentPose.nearest(blueReefFaces);
    }
  }

  public int getClosestReefFaceAprilTagId(Pose2d closestReefFace) {
    return this.reefFacePoseToAprilTagIdMap.get(closestReefFace);
  }

  public String getClosetReefFaceSmartDashboardName(Pose2d closestReefFace) {
    return this.reefFacePoseToTargetReefFace.get(closestReefFace).name();
  }

  public TargetReefFace getClosestReefFaceToTargetReefFace(Pose2d closestReefFace) {
    return this.reefFacePoseToTargetReefFace.get(closestReefFace);
  }

  private List<Pose2d> buildReefFaceList(boolean isRedAlliance) {
    if (isRedAlliance) {
      List<Pose2d> redFaces = new ArrayList<>();
      for (Pose2d face : FieldConstants.Reef.centerFaces) {
        redFaces.add(face.rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg));
      }
      return redFaces;
    } else {
      return List.of(FieldConstants.Reef.centerFaces);
    }
  }

  private Map<Pose2d, Integer> buildReefFacePoseToAprilTagMap() {
    Map<Pose2d, Integer> retMap = new HashMap<>();
    // BLUE FACES
    retMap.put(FieldConstants.Reef.centerFaces[0], 18);
    retMap.put(FieldConstants.Reef.centerFaces[1], 19);
    retMap.put(FieldConstants.Reef.centerFaces[2], 20);
    retMap.put(FieldConstants.Reef.centerFaces[3], 21);
    retMap.put(FieldConstants.Reef.centerFaces[4], 22);
    retMap.put(FieldConstants.Reef.centerFaces[5], 17);
    // RED FACES
    retMap.put(
        FieldConstants.Reef.centerFaces[0].rotateAround(
            FieldConstants.fieldCenter, Rotation2d.k180deg),
        7);
    retMap.put(
        FieldConstants.Reef.centerFaces[1].rotateAround(
            FieldConstants.fieldCenter, Rotation2d.k180deg),
        6);
    retMap.put(
        FieldConstants.Reef.centerFaces[2].rotateAround(
            FieldConstants.fieldCenter, Rotation2d.k180deg),
        11);
    retMap.put(
        FieldConstants.Reef.centerFaces[3].rotateAround(
            FieldConstants.fieldCenter, Rotation2d.k180deg),
        10);
    retMap.put(
        FieldConstants.Reef.centerFaces[4].rotateAround(
            FieldConstants.fieldCenter, Rotation2d.k180deg),
        9);
    retMap.put(
        FieldConstants.Reef.centerFaces[5].rotateAround(
            FieldConstants.fieldCenter, Rotation2d.k180deg),
        8);
    return retMap;
  }

  private Map<Pose2d, TargetReefFace> buildReefFacePoseToTarget() {
    Map<Pose2d, TargetReefFace> retMap = new HashMap<>();
    // BLUE FACES
    retMap.put(FieldConstants.Reef.centerFaces[0], TargetReefFace.A);
    retMap.put(FieldConstants.Reef.centerFaces[1], TargetReefFace.B);
    retMap.put(FieldConstants.Reef.centerFaces[2], TargetReefFace.C);
    retMap.put(FieldConstants.Reef.centerFaces[3], TargetReefFace.D);
    retMap.put(FieldConstants.Reef.centerFaces[4], TargetReefFace.E);
    retMap.put(FieldConstants.Reef.centerFaces[5], TargetReefFace.F);
    // RED FACES
    retMap.put(
        FieldConstants.Reef.centerFaces[0].rotateAround(
            FieldConstants.fieldCenter, Rotation2d.k180deg),
        TargetReefFace.A);
    retMap.put(
        FieldConstants.Reef.centerFaces[1].rotateAround(
            FieldConstants.fieldCenter, Rotation2d.k180deg),
        TargetReefFace.B);
    retMap.put(
        FieldConstants.Reef.centerFaces[2].rotateAround(
            FieldConstants.fieldCenter, Rotation2d.k180deg),
        TargetReefFace.C);
    retMap.put(
        FieldConstants.Reef.centerFaces[3].rotateAround(
            FieldConstants.fieldCenter, Rotation2d.k180deg),
        TargetReefFace.D);
    retMap.put(
        FieldConstants.Reef.centerFaces[4].rotateAround(
            FieldConstants.fieldCenter, Rotation2d.k180deg),
        TargetReefFace.E);
    retMap.put(
        FieldConstants.Reef.centerFaces[5].rotateAround(
            FieldConstants.fieldCenter, Rotation2d.k180deg),
        TargetReefFace.F);
    return retMap;
  }

  private Map<DesiredReefPosition, Pose2d> buildReefMap() {

    FieldConstants.Reef reef = new FieldConstants.Reef();
    //       3 d
    //       __
    // 2 c  /  \ 4 e
    // 1 b  \__/ 5 f
    //       0 a
    Map<DesiredReefPosition, Pose2d> reefMap = new HashMap<>();

    DesiredReefPosition aLeft = new DesiredReefPosition(0, BranchLocation.LEFT);
    DesiredReefPosition aRight = new DesiredReefPosition(0, BranchLocation.RIGHT);
    DesiredReefPosition aCenter = new DesiredReefPosition(0, BranchLocation.CENTER);

    DesiredReefPosition bLeft = new DesiredReefPosition(1, BranchLocation.LEFT);
    DesiredReefPosition bRight = new DesiredReefPosition(1, BranchLocation.RIGHT);
    DesiredReefPosition bCenter = new DesiredReefPosition(1, BranchLocation.CENTER);

    DesiredReefPosition cLeft = new DesiredReefPosition(2, BranchLocation.LEFT);
    DesiredReefPosition cRight = new DesiredReefPosition(2, BranchLocation.RIGHT);
    DesiredReefPosition cCenter = new DesiredReefPosition(2, BranchLocation.CENTER);

    DesiredReefPosition dLeft = new DesiredReefPosition(3, BranchLocation.LEFT);
    DesiredReefPosition dRight = new DesiredReefPosition(3, BranchLocation.RIGHT);
    DesiredReefPosition dCenter = new DesiredReefPosition(3, BranchLocation.CENTER);

    DesiredReefPosition eLeft = new DesiredReefPosition(4, BranchLocation.LEFT);
    DesiredReefPosition eRight = new DesiredReefPosition(4, BranchLocation.RIGHT);
    DesiredReefPosition eCenter = new DesiredReefPosition(4, BranchLocation.CENTER);

    DesiredReefPosition fLeft = new DesiredReefPosition(5, BranchLocation.LEFT);
    DesiredReefPosition fRight = new DesiredReefPosition(5, BranchLocation.RIGHT);
    DesiredReefPosition fCenter = new DesiredReefPosition(5, BranchLocation.CENTER);

    // lower number always on right for field constants
    //       78
    //       __
    // 6 5  /  \ 9 10
    // 4 3  \__/ 11 12
    //       2 1
    reefMap.put(
        aLeft,
        FieldConstants.Reef.branchPositions.get(1).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(
        aRight,
        FieldConstants.Reef.branchPositions.get(0).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(aCenter, FieldConstants.Reef.centerFaces[0]);

    reefMap.put(
        bLeft,
        FieldConstants.Reef.branchPositions.get(3).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(
        bRight,
        FieldConstants.Reef.branchPositions.get(2).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(bCenter, FieldConstants.Reef.centerFaces[1]);

    reefMap.put(
        cLeft,
        FieldConstants.Reef.branchPositions.get(4).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(
        cRight,
        FieldConstants.Reef.branchPositions.get(5).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(cCenter, FieldConstants.Reef.centerFaces[2]);

    reefMap.put(
        dLeft,
        FieldConstants.Reef.branchPositions.get(6).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(
        dRight,
        FieldConstants.Reef.branchPositions.get(7).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(dCenter, FieldConstants.Reef.centerFaces[3]);

    reefMap.put(
        eLeft,
        FieldConstants.Reef.branchPositions.get(8).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(
        eRight,
        FieldConstants.Reef.branchPositions.get(9).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(eCenter, FieldConstants.Reef.centerFaces[4]);

    reefMap.put(
        fLeft,
        FieldConstants.Reef.branchPositions.get(11).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(
        fRight,
        FieldConstants.Reef.branchPositions.get(10).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(fCenter, FieldConstants.Reef.centerFaces[5]);

    Logger.recordOutput("aLeft", reefMap.get(aLeft));
    Logger.recordOutput("aRight", reefMap.get(aRight));
    Logger.recordOutput("aCenter", reefMap.get(aCenter));

    Logger.recordOutput("bLeft", reefMap.get(bLeft));
    Logger.recordOutput("bRight", reefMap.get(bRight));
    Logger.recordOutput("bCenter", reefMap.get(bCenter));

    Logger.recordOutput("cLeft", reefMap.get(cLeft));
    Logger.recordOutput("cRight", reefMap.get(cRight));
    Logger.recordOutput("cCenter", reefMap.get(cCenter));

    Logger.recordOutput("dLeft", reefMap.get(dLeft));
    Logger.recordOutput("dRight", reefMap.get(dRight));
    Logger.recordOutput("dCenter", reefMap.get(dCenter));

    Logger.recordOutput("eLeft", reefMap.get(eLeft));
    Logger.recordOutput("eRight", reefMap.get(eRight));
    Logger.recordOutput("eCenter", reefMap.get(eCenter));

    Logger.recordOutput("fLeft", reefMap.get(fLeft));
    Logger.recordOutput("fRight", reefMap.get(fRight));
    Logger.recordOutput("fCenter", reefMap.get(fCenter));

    return reefMap;
  }
}
