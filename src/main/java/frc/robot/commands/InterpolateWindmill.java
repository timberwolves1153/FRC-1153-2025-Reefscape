package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.Interpolation.InterpolatingDouble;
import frc.robot.Interpolation.WindmillTable;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.windmill.Windmill;

public class InterpolateWindmill extends Command {

  private Windmill windmill;
  private Drive drive;
  private WindmillTable windmillTable;

  public InterpolateWindmill(Windmill windmill, Drive drive) {

    this.windmill = windmill;
    this.drive = drive;
    windmillTable = new WindmillTable();
  }

  @Override
  public void execute() {
    windmill.setTargetPositionDegrees(
        windmillTable.windmillMap.getInterpolated(
                new InterpolatingDouble(FieldConstants.getNearestCage(drive.getPose())))
            .value);
  }
}
