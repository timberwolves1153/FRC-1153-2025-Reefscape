package frc.robot.subsystems.alignment;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.HashSet;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class AlignmentIOPhotonVision implements AlignmentIO {

  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  /**
   * Creates a new AlignmentIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public AlignmentIOPhotonVision(String name, Transform3d robotToCamera) {

    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(AlignmentIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIDs = new HashSet<>();
    for (var result : camera.getAllUnreadResults()) {
      if (!result.targets.isEmpty()) { // single tag result
        var target = result.getBestTarget();
        // Add tag ID
        tagIDs.add((short) target.fiducialId);
        inputs.bestTargetTagId = target.getFiducialId();
        inputs.targetYaw = target.getYaw();
        inputs.targetRange =
            PhotonUtils.calculateDistanceToTargetMeters(
                0.5, // Measured with a tape measure, or in CAD.
                AlignmentConstants.REEF_TARGET_HEIGHT, // From 2024 game manual for ID 7
                Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                Units.degreesToRadians(target.getPitch()));
      }
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIDs.size()];
    int i = 0;
    for (int id : tagIDs) {
      inputs.tagIds[i++] = id;
    }
  }
}
