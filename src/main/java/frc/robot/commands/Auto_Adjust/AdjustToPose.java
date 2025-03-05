package frc.robot.commands.Auto_Adjust;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class AdjustToPose extends Command {

  private Drive drive;
  private static ChassisSpeeds robotSpeeds;
  public static AngleAutoAdjustController angleAdjustController;
  public static TranslationAutoAdjustController autoAdjustXYController;
  private Pose2d targetPose;
  private Supplier<Pose2d> isAtGoalPoseSupplier;
  private boolean useScoringTolerance = false;

  public AdjustToPose(Pose2d targetPose, Drive drive, Supplier<Pose2d> isAtGoalPoseSupplier) {
    this.drive = drive;
    this.targetPose = targetPose;
    this.isAtGoalPoseSupplier = isAtGoalPoseSupplier;

    angleAdjustController =
        new AngleAutoAdjustController(
            () -> drive.rawGyroRotation.getDegrees(), targetPose.getRotation().getDegrees());

    autoAdjustXYController =
        new TranslationAutoAdjustController(
            () -> drive.getPose(), () -> drive.getRotation().getDegrees(), () -> 0.0, targetPose);

    addRequirements(drive);
  }

  public void setTapePIDValues() {
    autoAdjustXYController.setPID(20, 0, 0, 0.01, 20, 0, 0, 0.05);
  }

  public void setScorePIDValues() {
    autoAdjustXYController.setPID(10, 0, 0, 0.01, 10, 0, 0, 0.05);
  }

  public void useScoringTolerance() {
    this.useScoringTolerance = true;
  }

  @Override
  public void execute() {
    robotSpeeds = autoAdjustXYController.update();
    robotSpeeds.omegaRadiansPerSecond = angleAdjustController.update().omegaRadiansPerSecond;

    drive.runVelocity(robotSpeeds);
  }

  @Override
  public boolean isFinished() {
    if (useScoringTolerance) {
      return (Math.abs(isAtGoalPoseSupplier.get().getX() - this.targetPose.getX()) < .025)
          && (Math.abs(isAtGoalPoseSupplier.get().getY() - this.targetPose.getY()) < .025);
    } else {
      return (Math.abs(isAtGoalPoseSupplier.get().getX() - this.targetPose.getX()) < .01)
          && (Math.abs(isAtGoalPoseSupplier.get().getY() - this.targetPose.getY()) < .01);
    }
  }
}
