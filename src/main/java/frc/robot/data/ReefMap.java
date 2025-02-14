package frc.robot.data;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.FieldConstants;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class ReefMap {

  private Map<DesiredReefPosition, Pose2d> map;

  public ReefMap() {
    this.map = buildReefMap();
  }

  public Map<DesiredReefPosition, Pose2d> getReefMap() {
    return this.map;
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
        FieldConstants.Reef.branchPositions.get(5).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(
        cRight,
        FieldConstants.Reef.branchPositions.get(4).get(FieldConstants.ReefHeight.L2).toPose2d());
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
        FieldConstants.Reef.branchPositions.get(10).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(
        fRight,
        FieldConstants.Reef.branchPositions.get(11).get(FieldConstants.ReefHeight.L2).toPose2d());
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
