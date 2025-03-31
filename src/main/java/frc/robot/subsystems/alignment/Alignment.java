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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Alignment extends SubsystemBase {
  private final AlignmentIO io;
  private final AlignmentIOInputsAutoLogged inputs;

  public Alignment(AlignmentIO io) {
    this.io = io;

    // Initialize inputs
    this.inputs = new AlignmentIOInputsAutoLogged();
  }

  public Transform3d getBestCameraToTarget() {
    if (AlignmentConstants.REEF_TAGS.contains(this.inputs.bestTargetTagId)) {
      return this.inputs.cameraToTarget;
    }
    return new Transform3d();
  }

  public int getTargetId() {
    return this.inputs.bestTargetTagId;
  }

  public Pose2d getRobotPose() {
    return inputs.photonpose.toPose2d();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Alignment/Camera", inputs);
  }
}
