package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.alignment.Alignment;
import frc.robot.subsystems.drive.Drive;

public class AlignCommand extends Command {

  private Alignment alignment;
  private Drive drive;

  private final double VISION_TURN_kP = 0.01;
  private final double VISION_DES_ANGLE_deg = 0.0;
  private final double VISION_FORWARD_kP = 0.5;
  private final double VISION_STRAFE_kP = 0.5;
  private final double VISION_DES_RANGE_m = 1.25;

  public AlignCommand(Alignment alignment, Drive drive) {
    this.alignment = alignment;
    this.drive = drive;

    addRequirements(alignment, drive);
  }

  @Override
  public void execute() {

    // double targetRange = alignment.getTargetDistance();
    // double targetYaw = alignment.getTargetYaw();
    // double turn =
    //     (VISION_DES_ANGLE_deg - targetYaw) * VISION_TURN_kP *
    // drive.getMaxAngularSpeedRadPerSec();

    // double forward = (-targetRange) * VISION_FORWARD_kP * drive.getMaxLinearSpeedMetersPerSec();
  }
}
