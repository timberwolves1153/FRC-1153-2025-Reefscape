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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Auto_Adjust.AdjustToPose;
import frc.robot.commands.DriveCommands;
import frc.robot.data.BranchLocation;
import frc.robot.data.DesiredReefPosition;
import frc.robot.data.ReefMap;
import frc.robot.generated.FinalTunerConstants;
import frc.robot.generated.ProtoTunerConstants;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIO;
import frc.robot.subsystems.Climber.ClimberIOSim;
import frc.robot.subsystems.Climber.ClimberIOSparkMax;
import frc.robot.subsystems.Manipulator.Algae;
import frc.robot.subsystems.Manipulator.AlgaeIO;
import frc.robot.subsystems.Manipulator.AlgaeIOSim;
import frc.robot.subsystems.Manipulator.AlgaeIOSparkMax;
import frc.robot.subsystems.Manipulator.Coral;
import frc.robot.subsystems.Manipulator.CoralIO;
import frc.robot.subsystems.Manipulator.CoralIOSim;
import frc.robot.subsystems.Manipulator.CoralIOSparkMax;
import frc.robot.subsystems.alignment.Alignment;
import frc.robot.subsystems.alignment.AlignmentConstants;
import frc.robot.subsystems.alignment.AlignmentIO;
import frc.robot.subsystems.alignment.AlignmentIOPhotonVision;
import frc.robot.subsystems.alignment.AlignmentIOPhotonVisionSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.TargetReefFace;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.windmill.Windmill;
import frc.robot.subsystems.windmill.WindmillIO;
import frc.robot.subsystems.windmill.WindmillIOSim;
import frc.robot.subsystems.windmill.WindmillIOTalonFX;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Windmill windmill;
  private final Elevator elevator;
  //   private final Superstructure superstructure;
  private final Coral coral;
  private final Algae algae;
  private final Vision vision;
  private final Climber climber;

  private final Alignment alignment;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick opBoard = new Joystick(1);
  private final CommandXboxController opOverride = new CommandXboxController(2);

  private final JoystickButton atariButton1 = new JoystickButton(opBoard, 1);
  private final JoystickButton atariButton2 = new JoystickButton(opBoard, 2);
  private final JoystickButton atariButton3 = new JoystickButton(opBoard, 3);
  private final JoystickButton atariButton4 = new JoystickButton(opBoard, 4);
  private final JoystickButton atariButton5 = new JoystickButton(opBoard, 5);
  private final JoystickButton atariButton6 = new JoystickButton(opBoard, 6);
  private final JoystickButton atariButton7 = new JoystickButton(opBoard, 7);
  private final JoystickButton atariButton8 = new JoystickButton(opBoard, 8);
  private final JoystickButton atariButton9 = new JoystickButton(opBoard, 9);
  private final JoystickButton atariButton10 = new JoystickButton(opBoard, 10);
  private final JoystickButton atariButton11 = new JoystickButton(opBoard, 11);
  private final JoystickButton atariButton12 = new JoystickButton(opBoard, 12);
  private final JoystickButton atariButton13 = new JoystickButton(opBoard, 13);
  private final JoystickButton atariButton14 = new JoystickButton(opBoard, 14);
  private final JoystickButton atariButton15 = new JoystickButton(opBoard, 15);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private Map<DesiredReefPosition, Pose2d> reefmap = new ReefMap().getReefMap();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putString("rioSerialNumber", HALUtil.getSerialNumber());
    SmartDashboard.putString("Current Robot", Constants.currentBot.name());
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        if (Constants.Bot.PROTO.equals(Constants.currentBot)) {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(ProtoTunerConstants.FrontLeft),
                  new ModuleIOTalonFX(ProtoTunerConstants.FrontRight),
                  new ModuleIOTalonFX(ProtoTunerConstants.BackLeft),
                  new ModuleIOTalonFX(ProtoTunerConstants.BackRight));
        } else {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(FinalTunerConstants.FrontLeft),
                  new ModuleIOTalonFX(FinalTunerConstants.FrontRight),
                  new ModuleIOTalonFX(FinalTunerConstants.BackLeft),
                  new ModuleIOTalonFX(FinalTunerConstants.BackRight));
        }
        windmill = new Windmill(new WindmillIOTalonFX());
        elevator = new Elevator(new ElevatorIOTalonFX());
        coral = new Coral(new CoralIOSparkMax());
        algae = new Algae(new AlgaeIOSparkMax());
        climber = new Climber(new ClimberIOSparkMax());
        // superstructure = new Superstructure(elevator, windmill, coral, algae);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1),
                new VisionIOPhotonVision(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2));
        alignment =
            new Alignment(
                new AlignmentIOPhotonVision(
                    AlignmentConstants.cameraName, AlignmentConstants.robotToCamera));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(FinalTunerConstants.FrontLeft),
                new ModuleIOSim(FinalTunerConstants.FrontRight),
                new ModuleIOSim(FinalTunerConstants.BackLeft),
                new ModuleIOSim(FinalTunerConstants.BackRight));

        windmill = new Windmill(new WindmillIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        coral = new Coral(new CoralIOSim());
        algae = new Algae(new AlgaeIOSim());
        // superstructure = new Superstructure(elevator, windmill, coral, algae);
        climber = new Climber(new ClimberIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2, drive::getPose));
        alignment =
            new Alignment(
                new AlignmentIOPhotonVisionSim(
                    AlignmentConstants.cameraName,
                    AlignmentConstants.robotToCamera,
                    drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        windmill = new Windmill(new WindmillIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        coral = new Coral(new CoralIO() {});
        algae = new Algae(new AlgaeIO() {});
        climber = new Climber(new ClimberIO() {});
        // superstructure = new Superstructure(elevator, windmill, coral, algae);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});

        alignment = new Alignment(new AlignmentIO() {});
        break;
    }

    NamedCommands.registerCommand("Intake Coral", new InstantCommand(() -> coral.runVolts(-6.5)));
    NamedCommands.registerCommand("reset gyro 180", new InstantCommand(() -> drive.resetGyro(180)));
    NamedCommands.registerCommand("reset gyro 0", new InstantCommand(() -> drive.resetGyro(0)));
    NamedCommands.registerCommand("reset gyro -60", new InstantCommand(() -> drive.resetGyro(-60)));
    NamedCommands.registerCommand("reset gyro 60", new InstantCommand(() -> drive.resetGyro(60)));
    NamedCommands.registerCommand("Hold Coral", new InstantCommand(() -> coral.runVolts(-2)));
    NamedCommands.registerCommand("Stop Coral", new InstantCommand(() -> coral.runVolts(0)));
    NamedCommands.registerCommand("Outtake Coral", new InstantCommand(() -> coral.runVolts(5)));
    NamedCommands.registerCommand(
        "quick right align", DriveCommands.alignToReefFace(false, drive).withTimeout(0.5));
    NamedCommands.registerCommand(
        "Stop Algae Inner", new InstantCommand(() -> algae.setVoltageHolding(0)));
    NamedCommands.registerCommand(
        "Slow Algae Inner", new InstantCommand(() -> algae.setVoltageHolding(2)));
    NamedCommands.registerCommand(
        "Stop Algae Outer", new InstantCommand(() -> algae.setVoltageLauncher(0)));

    NamedCommands.registerCommand(
        "Grab Algae Inner", new InstantCommand(() -> algae.setVoltageHolding(6)));
    NamedCommands.registerCommand(
        "Grab Algae Outer", new InstantCommand(() -> algae.setVoltageLauncher(-6)));

    NamedCommands.registerCommand(
        "Shoot Algae Inner", new InstantCommand(() -> algae.setVoltageHolding(-6)));
    NamedCommands.registerCommand(
        "Shoot Algae Outer", new InstantCommand(() -> algae.setVoltageLauncher(12)));

    // NamedCommands.registerCommand(
    //     "Stow Position", Commands.runOnce(() -> superstructure.setAutoGoalCommand(Goal.STOW)));
    // NamedCommands.registerCommand(
    //     "Collect Coral Position", superstructure.setGoalCommand(Goal.COLLECT));
    // NamedCommands.registerCommand(
    //     "Coral Mode",
    //     Commands.runOnce(
    //         () -> superstructure.setAutoGamepieceCommand(GamePiece.CORAL), superstructure));
    // NamedCommands.registerCommand(
    //     "Algae Mode",
    //     Commands.runOnce(
    //         () -> superstructure.setAutoGamepieceCommand(GamePiece.ALGAE), superstructure));
    // NamedCommands.registerCommand(
    //     "Score L1 Coral Position",
    //     Commands.runOnce(() -> superstructure.setAutoGoalCommand(Goal.L1), superstructure));
    // // NamedCommands.registerCommand(
    // //     "Stay At Previous Position",
    // //     superstructure.setAutoGoalCommand(superstructure.getCurrentGoal()));
    // NamedCommands.registerCommand(
    //     "L2 Position",
    //     Commands.runOnce(() -> superstructure.setAutoGoalCommand(Goal.L2), superstructure));
    // NamedCommands.registerCommand(
    //     "L3 Position",
    //     Commands.runOnce(() -> superstructure.setAutoGoalCommand(Goal.L3), superstructure));
    // NamedCommands.registerCommand(
    //     "Barge Position",
    //     Commands.runOnce(() -> superstructure.setAutoGoalCommand(Goal.BARGE), superstructure));

    NamedCommands.registerCommand(
        "Auto Align Center",
        new ConditionalCommand(
                alignThenScore(BranchLocation.CENTER),
                alignToScore(BranchLocation.CENTER, false),
                () -> isCloseToReef())
            .withTimeout(1));

    NamedCommands.registerCommand(
        "Auto Align Left",
        new ConditionalCommand(
                alignThenScore(BranchLocation.LEFT),
                alignToScore(BranchLocation.LEFT, false),
                () -> isCloseToReef())
            .withTimeout(1));

    NamedCommands.registerCommand(
        "Auto Align Right",
        new ConditionalCommand(
                alignThenScore(BranchLocation.RIGHT),
                alignToScore(BranchLocation.RIGHT, false),
                () -> isCloseToReef())
            .withTimeout(1));

    // NamedCommands.registerCommand("Auto Align Station", drive.driveToStation());

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // // Lock to 0° when A button is held
    controller
        .rightStick()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () ->
                    new Rotation2d(
                        FieldConstants.getNearestCoralStation(drive.getPose())
                            .getRotation()
                            .getRadians())));

    // controller.x().whileTrue(new AdjustToPose(FieldConstants.Reef.centerFaces[2], drive));
    // controller.b().whileTrue(drive.driveToBarge());

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when Back button is pressed
    controller
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller.back().onTrue(Commands.runOnce(() -> drive.resetGyro(0), drive));
    controller.back().onTrue(Commands.runOnce(() -> climber.zeroClimb(), climber));

    controller
        .leftBumper()
        .whileTrue(
            new ConditionalCommand(
                alignThenScore(BranchLocation.LEFT),
                alignToScore(BranchLocation.LEFT, false),
                () -> isCloseToReef()));
    // controller
    //     .leftBumper()
    //     .and(controller.leftStick())
    //     .whileTrue(
    //         new ConditionalCommand(
    //             alignThenScore(BranchLocation.LEFT),
    //             alignToScore(BranchLocation.LEFT, true),
    //             () -> isCloseToReef()));
    // controller
    //     .rightBumper()
    //     .and(controller.leftStick())
    //     .whileTrue(
    //         new ConditionalCommand(
    //             alignThenScore(BranchLocation.RIGHT),
    //             alignToScore(BranchLocation.RIGHT, true),
    //             () -> isCloseToReef()));
    //
    // controller.rightBumper().whileTrue(alignToTape().andThen(alignToScore(BranchLocation.RIGHT)));
    controller
        .rightBumper()
        .whileTrue(
            new ConditionalCommand(
                alignThenScore(BranchLocation.RIGHT),
                alignToScore(BranchLocation.RIGHT, false),
                () -> isCloseToReef()));
    //    controller.a().whileTrue(alignToTape().andThen(alignToScore(BranchLocation.LEFT)));
    controller
        .a()
        .whileTrue(
            new ConditionalCommand(
                alignThenScore(BranchLocation.CENTER),
                alignToScore(BranchLocation.CENTER, false),
                () -> isCloseToReef()));
    // controller.b().whileTrue(alignToTape().andThen(alignToScore(BranchLocation.LEFT)));

    controller.pov(0).onTrue(new InstantCommand(() -> climber.setVoltage(-12)));
    controller.pov(0).onFalse(new InstantCommand(() -> climber.setVoltage(0)));

    controller.pov(180).onTrue(new InstantCommand(() -> climber.setVoltage(12)));
    controller.pov(180).onFalse(new InstantCommand(() -> climber.setVoltage(0)));
    controller.pov(270).whileTrue(DriveCommands.alignToReefFace(true, drive));
    controller.pov(90).whileTrue(DriveCommands.alignToReefFace(false, drive));
    controller.leftTrigger().onTrue(Commands.runOnce(() -> climber.setPosition(-70)));
    controller.rightTrigger().onTrue(Commands.runOnce(() -> climber.setPosition(170)));

    controller.b().whileTrue(alignToScore(BranchLocation.CENTER, false));

    // atariButton9.onTrue(new InstantCommand(() -> drive.setDesiredReefFace(TargetReefFace.A)));
    // atariButton10.onTrue(new InstantCommand(() -> drive.setDesiredReefFace(TargetReefFace.B)));

    // atariButton11.onTrue(new InstantCommand(() -> drive.setDesiredReefFace(TargetReefFace.C)));
    // atariButton12.onTrue(new InstantCommand(() -> drive.setDesiredReefFace(TargetReefFace.D)));
    // atariButton14.onTrue(new InstantCommand(() -> drive.setDesiredReefFace(TargetReefFace.E)));
    // atariButton15.onTrue(new InstantCommand(() -> drive.setDesiredReefFace(TargetReefFace.F)));

    // controller.y().onTrue(new InstantCommand(() -> elevator.setVoltage(3)));
    // controller.y().onFalse(new InstantCommand(() -> elevator.setVoltage(0.25)));

    // controller.start().onTrue(new InstantCommand(() -> elevator.setVoltage(-3)));
    // controller.start().onFalse(new InstantCommand(() -> elevator.setVoltage(0.25)));

    // controller.x().onTrue(new InstantCommand(() -> windmill.setVoltage(3)));
    // controller.x().onFalse(new InstantCommand(() -> windmill.setVoltage(0)));

    // controller.b().onTrue(new InstantCommand(() -> windmill.setVoltage(-3)));
    // controller.b().onFalse(new InstantCommand(() -> windmill.setVoltage(0)));

    // controller.rightTrigger().onTrue(new InstantCommand(() -> algae.setVoltageHolding(-6)));
    // controller.rightTrigger().onFalse(new InstantCommand(() -> algae.setVoltageHolding(0)));

    // controller.rightTrigger().onTrue(new InstantCommand(() -> coral.runVolts(-5)));
    // controller.rightTrigger().onFalse(new InstantCommand(() -> coral.runVolts(0)));

    // controller.leftTrigger().onTrue(new InstantCommand(() -> coral.runVolts(5)));
    // controller.leftTrigger().onFalse(new InstantCommand(() -> coral.runVolts(0)));

    // controller.rightStick().onTrue(new InstantCommand(() -> algae.setVoltageLauncher(12)));
    // controller.rightStick().onFalse(new InstantCommand(() -> algae.setVoltageLauncher(0)));

    // controller.leftTrigger().onTrue(new InstantCommand(() -> algae.setVoltageHolding(6)));
    // controller.leftTrigger().onFalse(new InstantCommand(() -> algae.setVoltageHolding(0)));

    // controller.leftTrigger().onTrue(new InstantCommand(() -> algae.setVoltageLauncher(-6)));
    // controller.leftTrigger().onFalse(new InstantCommand(() -> algae.setVoltageLauncher(0)));

    // if (atariButton13.getAsBoolean()) {
    //   coral.setCurrentGamePiece(GamePiece.ALGAE);

    // } else {

    //   coral.setCurrentGamePiece(GamePiece.CORAL);
    // }
    // atariButton13.onTrue(superstructure.setGamepieceCommand(GamePiece.ALGAE));
    // atariButton13.onFalse(superstructure.setGamepieceCommand(GamePiece.CORAL));
    // controller.leftTrigger().onTrue(superstructure.setGoalCommand(Goal.CLIMB));

    // //  controller.x().whileTrue(drive.driveToStation());
    // // // controller.b().whileTrue(drive.driveToBarge());whd(Goal.STOW));
    // atariButton1.onTrue(superstructure.setGoalCommand(Goal.STOW));
    // atariButton2.onTrue(superstructure.setGoalCommand(Goal.L1));
    // atariButton3.onTrue(superstructure.setGoalCommand(Goal.L2));
    // atariButton4.onTrue(superstructure.setGoalCommand(Goal.L3));
    // atariButton5.onTrue(superstructure.setGoalCommand(Goal.BARGE));
    // atariButton6.onTrue(superstructure.setGoalCommand(Goal.COLLECT));
    // atariButton8.whileTrue(new CollectGamePiece(coral, algae, superstructure));
    // atariButton8.whileFalse(
    //     new ConditionalCommand(
    //         new JiggleCoral(coral),
    //         new InstantCommand(() -> coral.stop()),
    //         () -> GamePiece.CORAL.equals(superstructure.getGamePiece())));
    // atariButton7.whileTrue(new ScoreGamePiece(coral, algae, superstructure));
  }

  public boolean isCloseToReef() {
    TargetReefFace reefFace = drive.getDesiredReefFace();
    DesiredReefPosition goalPosition =
        new DesiredReefPosition(reefFace.faceNumber, BranchLocation.CENTER);
    Pose2d closestTagPose = reefmap.get(goalPosition);
    double dist = drive.getPose().getTranslation().getDistance(closestTagPose.getTranslation());
    return Math.abs(dist) < Units.inchesToMeters(24);
  }

  public Command alignThenScore(BranchLocation location) {
    return alignToTape().andThen(alignToScore(location, false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void stopIntakes() {
    algae.stopHolding();
    algae.stopLauncher();
    coral.stop();
  }

  public Command driveToReef(Supplier<TargetReefFace> desiredFace, BranchLocation desiredLocation) {
    // FieldConstants.getNearestReefFace(drive.getPose());
    // List<Map<ReefHeight branchPositions = FieldConstants.Reef.branchPositions;
    //            6   7
    //         5        8
    //        4          9
    //         3        10
    //          2     11
    //            1 0

    return new DeferredCommand(
        () -> {
          DesiredReefPosition goalPosition =
              new DesiredReefPosition(desiredFace.get().faceNumber, desiredLocation);
          SmartDashboard.putNumber("desiredPosition Face", desiredFace.get().faceNumber);
          SmartDashboard.putNumber("goalPosition Face", goalPosition.getFace());

          Transform2d desiredGamepieceTransform;
          if (BranchLocation.CENTER.equals(desiredLocation)) {
            desiredGamepieceTransform = Constants.ALGAE_TRANSFORM;
          } else {
            desiredGamepieceTransform = Constants.CORAL_TRANSFORM;
          }

          Pose2d goalPose = reefmap.get(goalPosition);
          Logger.recordOutput(
              "Auto Drive Target Pose",
              goalPose
                  .transformBy(Constants.ROBOT_TRANSFORM)
                  .rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg));
          boolean isRedAlliance =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          if (isRedAlliance) {
            // return AutoBuilder.pathfindToPose(
            //     goalPose
            //         .transformBy(robotTransform)
            //         .transformBy(desiredGamepieceTransform)
            //         .rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg),
            //     constraints);
            // drive.setPose(vision.getReefCameraPose());
            drive.setPose(alignment.getRobotPose());
            return new AdjustToPose(
                goalPose
                    .transformBy(Constants.ROBOT_TRANSFORM)
                    .transformBy(desiredGamepieceTransform)
                    .rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg),
                drive,
                alignment::getRobotPose);
          } else {
            // AutoBuilder.pathfindToPose(goalPose.transformBy(robotTransform), constraints);
            return new AdjustToPose(
                goalPose.transformBy(Constants.ROBOT_TRANSFORM), drive, alignment::getRobotPose);
          }
        },
        Set.of(drive));
  }

  public Command alignToTape() {
    // FieldConstants.getNearestReefFace(drive.getPose());
    // List<Map<ReefHeight branchPositions = FieldConstants.Reef.branchPositions;
    //            6   7
    //         5        8
    //        4          9
    //         3        10
    //          2     11
    //            1 0

    return new DeferredCommand(
        () -> {
          TargetReefFace desiredReefFace = drive.getDesiredReefFace();
          DesiredReefPosition goalPosition =
              new DesiredReefPosition(desiredReefFace.faceNumber, BranchLocation.CENTER);
          SmartDashboard.putNumber("desiredPosition Face", desiredReefFace.faceNumber);
          SmartDashboard.putNumber("goalPosition Face", goalPosition.getFace());

          Pose2d goalPose = reefmap.get(goalPosition);
          boolean isRedAlliance =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          if (isRedAlliance) {
            // return AutoBuilder.pathfindToPose(
            //     goalPose
            //         .transformBy(robotTransform)
            //         .transformBy(desiredGamepieceTransform)
            //         .rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg),
            //     constraints);
            // drive.setPose(vision.getReefCameraPose());
            Logger.recordOutput(
                "Auto Drive Target Pose",
                goalPose
                    .transformBy(Constants.ROBOT_TRANSFORM)
                    .transformBy(Constants.AUTOALIGN_TRANSFORM)
                    .rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg));
            if (DriverStation.isAutonomous()) {
              Command command =
                  AutoBuilder.pathfindToPose(
                      goalPose
                          .transformBy(Constants.AUTO_ROBOT_TRANSFORM)
                          .transformBy(Constants.AUTOALIGN_TRANSFORM)
                          .rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg),
                      drive.constraints);
              return command;
            } else {
              Command command =
                  AutoBuilder.pathfindToPose(
                      goalPose
                          .transformBy(Constants.ROBOT_TRANSFORM)
                          .transformBy(Constants.AUTOALIGN_TRANSFORM)
                          .rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg),
                      drive.constraints);
              return command;
            }
            // command.setTapePIDValues();

          } else { // BLUE
            // AutoBuilder.pathfindToPose(goalPose.transformBy(robotTransform), constraints);
            Logger.recordOutput(
                "Auto Drive Target Pose",
                goalPose
                    .transformBy(Constants.ROBOT_TRANSFORM)
                    .transformBy(Constants.AUTOALIGN_TRANSFORM));
            if (DriverStation.isAutonomous()) {
              // AUTO
              Command command =
                  AutoBuilder.pathfindToPose(
                      goalPose
                          .transformBy(Constants.AUTO_ROBOT_TRANSFORM)
                          .transformBy(Constants.AUTOALIGN_TRANSFORM)
                          .transformBy(new Transform2d(0, 0, Rotation2d.k180deg)),
                      drive.constraints);
              // command.setTapePIDValues();
              return command;
            } else {
              // TELE
              Command command =
                  AutoBuilder.pathfindToPose(
                      goalPose
                          .transformBy(Constants.ROBOT_TRANSFORM)
                          .transformBy(Constants.AUTOALIGN_TRANSFORM),
                      drive.constraints);
              // command.setTapePIDValues();
              return command;
            }
          }
        },
        Set.of(drive));
  }

  public Command alignToScore(BranchLocation branchLocation, boolean isL4) {
    return new DeferredCommand(
        () -> {
          TargetReefFace desiredReefFace = drive.getDesiredReefFace();
          DesiredReefPosition goalPosition =
              new DesiredReefPosition(desiredReefFace.faceNumber, branchLocation);
          SmartDashboard.putNumber("desiredPosition Face", desiredReefFace.faceNumber);
          SmartDashboard.putNumber("goalPosition Face", goalPosition.getFace());

          Transform2d algaeTransform = new Transform2d(0, 0, new Rotation2d());

          if (branchLocation.equals(BranchLocation.CENTER)) {
            algaeTransform = Constants.ALGAE_TRANSFORM;
          } else {
            algaeTransform = algaeTransform;
          }

          Transform2d l4Transform = new Transform2d(0, 0, new Rotation2d());

          if (isL4) {
            l4Transform = Constants.L4_TRANSFORM;
          } else {
            l4Transform = l4Transform;
          }

          Pose2d goalPose = reefmap.get(goalPosition);
          boolean isRedAlliance =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.setPose(alignment.getRobotPose());
          if (isRedAlliance) {
            Logger.recordOutput(
                "Auto Drive Target Pose",
                goalPose
                    .transformBy(Constants.ROBOT_TRANSFORM)
                    .rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg));
            if (DriverStation.isAutonomous()) {
              AdjustToPose command =
                  new AdjustToPose(
                      goalPose
                          .transformBy(Constants.AUTO_ROBOT_TRANSFORM)
                          .transformBy(l4Transform)
                          .rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg)
                          .transformBy(new Transform2d(0, 0, Rotation2d.k180deg)),
                      drive,
                      alignment::getRobotPose);
              command.setScorePIDValues();
              command.useScoringTolerance();
              return command;
            } else {
              AdjustToPose command =
                  new AdjustToPose(
                      goalPose
                          .transformBy(Constants.ROBOT_TRANSFORM)
                          .transformBy(l4Transform)
                          .rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg)
                          .transformBy(new Transform2d(0, 0, Rotation2d.k180deg)),
                      drive,
                      alignment::getRobotPose);
              command.setScorePIDValues();
              command.useScoringTolerance();
              return command;
            }

          } else {
            // BLUE
            Logger.recordOutput(
                "Auto Drive Target Pose", goalPose.transformBy(Constants.ROBOT_TRANSFORM));

            if (DriverStation.isAutonomous()) {
              AdjustToPose command =
                  new AdjustToPose(
                      goalPose
                          .transformBy(Constants.AUTO_ROBOT_TRANSFORM)
                          .transformBy(new Transform2d(0, 0, Rotation2d.k180deg)),
                      drive,
                      alignment::getRobotPose);
              command.setScorePIDValues();
              command.useScoringTolerance();
              return command;
            } else {
              AdjustToPose command =
                  new AdjustToPose(
                      goalPose.transformBy(Constants.ROBOT_TRANSFORM),
                      drive,
                      alignment::getRobotPose);
              command.setScorePIDValues();
              command.useScoringTolerance();
              return command;
            }
          }
        },
        Set.of(drive));
  }

  public void calibration() {
    TargetReefFace desiredReefFace = drive.getDesiredReefFace();
    DesiredReefPosition goalPosition =
        new DesiredReefPosition(desiredReefFace.faceNumber, BranchLocation.CENTER);
    SmartDashboard.putNumber("desiredPosition Face", desiredReefFace.faceNumber);
    SmartDashboard.putNumber("goalPosition Face", goalPosition.getFace());

    Transform2d algaeTransform = new Transform2d(0, 0, new Rotation2d());

    Pose2d goalPose = reefmap.get(goalPosition);
    boolean isRedAlliance =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.setPose(alignment.getRobotPose());
    if (isRedAlliance) {
      Logger.recordOutput(
          "Calibration Target Pose",
          goalPose
              .transformBy(Constants.ROBOT_TRANSFORM)
              .rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg)
              .transformBy(new Transform2d(0, 0, Rotation2d.k180deg)));
      AdjustToPose command =
          new AdjustToPose(
              goalPose
                  .transformBy(Constants.ROBOT_TRANSFORM)
                  .rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg)
                  .transformBy(new Transform2d(0, 0, Rotation2d.k180deg)),
              drive,
              alignment::getRobotPose);
    } else {
      // AutoBuilder.pathfindToPose(goalPose.transformBy(robotTransform), constraints);
      Logger.recordOutput(
          "Calibration Target Pose", goalPose.transformBy(Constants.ROBOT_TRANSFORM));
    }
  }

  //   public Command alignToScore() {
  //     return new DeferredCommand(
  //         () -> {
  //           Transform3d goalTransform = alignment.getBestCameraToTarget();
  //           //          TargetReefFace desiredReefFace = drive.getDesiredReefFace();
  //           //          Pose2d reefFacePose =
  //           // FieldConstants.Reef.centerFaces[desiredReefFace.faceNumber];

  //           Transform2d goalTransform2d =
  //               new Transform2d(goalTransform.getX(), goalTransform.getY(), new Rotation2d(180));
  //           Pose2d goalPose = drive.getPose().transformBy(goalTransform2d);
  //           Logger.recordOutput(
  //               "Align To Score Target Pose", goalPose); //
  // .transformBy(Constants.ROBOT_TRANSFORM)
  //           boolean isRedAlliance =
  //               DriverStation.getAlliance().isPresent()
  //                   && DriverStation.getAlliance().get() == Alliance.Red;
  //           // AutoBuilder.pathfindToPose(goalPose.transformBy(robotTransform), constraints);
  //           // return new AdjustToPose(goalPose, drive);
  //           return new AdjustToPose(goalPose, drive);
  //         },
  //         Set.of(drive));
  //   }
}
