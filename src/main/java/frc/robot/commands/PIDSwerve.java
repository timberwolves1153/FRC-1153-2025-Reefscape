package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class PIDSwerve extends Command {
  private final Drive s_Swerve;
  private final Pose2d targetPose;
  private final boolean precise;

  private final PIDController xPID = new PIDController(0.09, 0, 0); // TODO Make Constants
  private final PIDController yPID = new PIDController(0.09, 0, 0);
  private final double positionTolerance = 1.0; // inches
  private final double roughPositionTolerance = 2.5; // inches
  private final double maxSpeed = 5.41 / 3.0;
  private final double positionKS = 0.02;
  private final double positionIZone = 4.0;

  private final PIDController rotationPID = new PIDController(0.003, 0, 0);
  private final double rotationTolerance = 1.0; // degrees
  private final double roughRotatationTolerance = 2.5; // degrees
  private final double maxAngularVelocity =
      (maxSpeed / 13) / 2.0; // 1/2 of angular velocity (speed/drivebase radius)
  public static final double rotationKS = 0.02;
  public static final double rotationIZone = 2.0; // degrees

  private Pose2d currentPose;

  public PIDSwerve(Drive s_Swerve, Pose2d currentPose, Pose2d targetPose, boolean precise) {
    super();

    this.s_Swerve = s_Swerve;
    this.currentPose = currentPose;
    this.targetPose = targetPose;
    this.precise = precise;
    addRequirements(s_Swerve);

    xPID.setIZone(positionIZone); // Only use Integral term within this range
    xPID.setIntegratorRange(-positionKS * 2, positionKS * 2);
    xPID.setSetpoint(Units.metersToInches(targetPose.getX()));
    xPID.setTolerance(precise ? positionTolerance : roughPositionTolerance);

    yPID.setIZone(positionIZone); // Only use Integral term within this range
    yPID.setIntegratorRange(-positionKS * 2, positionKS * 2);
    yPID.setSetpoint(Units.metersToInches(targetPose.getY())); // TODO Set derivative, too
    yPID.setTolerance(precise ? positionTolerance : roughPositionTolerance);

    rotationPID.enableContinuousInput(-180.0, 180.0);
    rotationPID.setIZone(rotationIZone); // Only use Integral term within this range
    rotationPID.setIntegratorRange(-rotationKS * 2, rotationKS * 2);
    rotationPID.setSetpoint(targetPose.getRotation().getDegrees());
    rotationPID.setTolerance(
        precise ? rotationTolerance : roughRotatationTolerance); // TODO Set derivative, too
  }

  @Override
  public void initialize() {
    super.initialize();

    xPID.reset();
    yPID.reset();
    rotationPID.reset();
  }

  @Override
  public void execute() {
    Pose2d pose = currentPose;
    Translation2d position = pose.getTranslation();
    Rotation2d rotation = pose.getRotation();

    /* TODO Consider a potential need to rotate most of the way first, then translate */

    double xCorrection = xPID.calculate(position.getX(), targetPose.getX());
    double xFeedForward = positionKS * Math.signum(xCorrection);
    double xVal = MathUtil.clamp(xCorrection + xFeedForward, -1.0, 1.0);

    double yCorrection = yPID.calculate(position.getY(), targetPose.getY());
    double yFeedForward = positionKS * Math.signum(yCorrection);
    double yVal = MathUtil.clamp(yCorrection + yFeedForward, -1.0, 1.0);

    double correction =
        rotationPID.calculate(rotation.getRadians(), targetPose.getRotation().getRadians());
    double feedForward = rotationKS * Math.signum(correction);
    double rotationVal = MathUtil.clamp(correction + feedForward, -1.0, 1.0);

    /* Drive */

    Translation2d chassisTranslation = new Translation2d(xVal, yVal).times(maxSpeed);
    double chassisRotation = rotationVal * maxAngularVelocity;

    ChassisSpeeds desiredChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            chassisTranslation.getX(),
            chassisTranslation.getY(),
            chassisRotation,
            s_Swerve.getRotation());
    s_Swerve.runVelocity(desiredChassisSpeeds);
  }

  @Override
  public boolean isFinished() {
    return xPID.atSetpoint() && yPID.atSetpoint() && rotationPID.atSetpoint();
  }
}
