package frc.robot.commands.Auto_Adjust;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;

public class TranslationAutoAdjustController implements SwerveController {

  private Supplier<Pose2d> currentPoseSupplier;
  private PIDController xController = new PIDController(10, 0, 0.5);
  private PIDController yController = new PIDController(10, 0, 0.5);
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

  private boolean isFieldRelativ = true;
  private Supplier<Double> gyromMeasurment;
  private Supplier<Double> gyroOffset;
  private Pose2d targetPose;

  public TranslationAutoAdjustController(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Double> robotAngle,
      Supplier<Double> gyroOffset,
      Pose2d setPoint) {
    currentPoseSupplier = robotPoseSupplier;
    gyromMeasurment = robotAngle;
    this.gyroOffset = gyroOffset;

    xController.setTolerance(0.05);
    yController.setTolerance(0.05);

    updateSetPoint(setPoint);
    isFieldRelativ = true;
  }

  public TranslationAutoAdjustController(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Double> gyroOffset,
      Supplier<Double> robotAngle) {
    this(robotPoseSupplier, robotAngle, gyroOffset, new Pose2d());
  }

  public void setConstrains(Constraints constraints) {}

  public void setField(boolean field) {
    isFieldRelativ = field;
  }

  public void setPID(
      double xKp,
      double xKi,
      double xKd,
      double xTolernace,
      double yKp,
      double yKi,
      double yKd,
      double yTolernace) {
    yController.setPID(yKp, yKi, yKd);
    yController.setTolerance(yTolernace);
    xController.setPID(xKp, xKi, xKd);
    xController.setTolerance(xTolernace);
  }

  public ChassisSpeeds update() {
    chassisSpeeds.vxMetersPerSecond = xController.calculate(currentPoseSupplier.get().getX());
    chassisSpeeds.vyMetersPerSecond = yController.calculate(currentPoseSupplier.get().getY());

    if (isFieldRelativ && DriverStation.getAlliance().get() == Alliance.Blue) {
      return ChassisSpeedsUtil.FromFieldToRobot(
          chassisSpeeds,
          new Rotation2d(Math.toRadians((-(gyromMeasurment.get() - gyroOffset.get())))));
    } else if (isFieldRelativ) {
      return ChassisSpeedsUtil.FromFieldToRobot(
          chassisSpeeds,
          new Rotation2d(Math.toRadians(((gyromMeasurment.get() - gyroOffset.get())))));
      // - 180))));
    }

    return chassisSpeeds;
  }

  public void updateSetPoint(Pose2d setPoint) {
    xController.setSetpoint(setPoint.getX());
    yController.setSetpoint(setPoint.getY());

    targetPose = setPoint;
  }

  public void setTolerance(double tolerance) {
    xController.setTolerance(tolerance);
    yController.setTolerance(tolerance);
  }

  public Pose2d getSetPoint() {
    return targetPose;
  }

  public boolean atPoint() {
    return xController.atSetpoint() && yController.atSetpoint();
  }

  public void updateMeaurment(Supplier<Pose2d> newPose2d) {
    currentPoseSupplier = newPose2d;
  }
}
