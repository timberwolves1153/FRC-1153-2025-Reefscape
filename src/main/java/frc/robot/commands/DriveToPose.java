// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class DriveToPose extends Command {
  Drive m_drivetrain;
  double xSpeed;
  double ySpeed;
  double omegaSpeed;
  Pose2d robotPose;
  Translation2d targetTranslation = new Translation2d(0, 0);
  Rotation2d targetAngle = new Rotation2d(0);
  boolean isFinished;

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
      new TrapezoidProfile.Constraints(3, 2.5);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(3, 2.5);
  private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(3, 2.5);

  private final ProfiledPIDController xController =
      new ProfiledPIDController(1, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(1, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(1, 0, 0, THETA_CONSTRAINTS);

  // private final SwerveRequest.FieldCentricFacingAngle swerveRequestFacing = new
  // SwerveRequest.FieldCentricFacingAngle()
  // .withDriveRequestType(DriveRequestType.Velocity)
  // .withSteerRequestType(SteerRequestType.MotionMagic)
  // .withVelocityX(0.0)
  // .withVelocityY(0.0);

  public DriveToPose(Drive drivetrain, Translation2d target, Rotation2d angle) {
    m_drivetrain = drivetrain;
    targetTranslation = target;
    targetAngle = angle;
    xController.setTolerance(0.02);
    yController.setTolerance(0.02);
    thetaController.setTolerance(0.01);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putData("driveToPose xController", xController);
    SmartDashboard.putData("driveToPose yController", yController);
    SmartDashboard.putData("driveToPoseHead", thetaController);
    SmartDashboard.putBoolean("driveToPose xController at Target", xController.atGoal());
    SmartDashboard.putBoolean("driveToPose yController at Target", yController.atGoal());
    SmartDashboard.putBoolean("driveToPose theta controller at Target", thetaController.atGoal());

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println(targetTranslation);
    // System.out.println(targetAngle);
    robotPose = m_drivetrain.getPose();
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    thetaController.reset(robotPose.getRotation().getRadians());
    xController.setGoal(targetTranslation.getX());
    yController.setGoal(targetTranslation.getY());
    thetaController.setGoal(targetAngle.getRadians());
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xController.setGoal(targetTranslation.getX());
    yController.setGoal(targetTranslation.getY());
    thetaController.setGoal(targetAngle.getRadians());
    robotPose = m_drivetrain.getPose();

    xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
    }

    m_drivetrain.runVelocity(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void setTarget(Translation2d pose) {
    targetTranslation = pose;
  }

  public void setAngle(Rotation2d angle) {
    targetAngle = angle;
  }
}
