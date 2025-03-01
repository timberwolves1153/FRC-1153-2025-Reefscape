package frc.robot.commands.Auto_Adjust;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class AdjustToPose extends Command {

  private Drive drive;
  private static ChassisSpeeds robotSpeeds;
  public static AngleAutoAdjustController angleAdjustController;
  public static TranslationAutoAdjustController autoAdjustXYController;
  private Pose2d targetPose;

  public AdjustToPose(Pose2d targetPose, Drive drive) {
    this.drive = drive;
    this.targetPose = targetPose;

    angleAdjustController =
        new AngleAutoAdjustController(
            () -> drive.rawGyroRotation.getDegrees(), targetPose.getRotation().getDegrees());

    autoAdjustXYController =
        new TranslationAutoAdjustController(
            () -> drive.getPose(), () -> drive.getRotation().getDegrees(), () -> 0.0, targetPose);

    addRequirements(drive);
  }

  @Override
  public void execute() {
    robotSpeeds = autoAdjustXYController.update();
    robotSpeeds.omegaRadiansPerSecond = angleAdjustController.update().omegaRadiansPerSecond;

    drive.runVelocity(robotSpeeds);
  }

  @Override
  public boolean isFinished() {
    return autoAdjustXYController.atPoint();
  }
}
