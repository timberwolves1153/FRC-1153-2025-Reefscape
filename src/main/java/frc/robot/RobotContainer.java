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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.Constants.GamePiece;
import frc.robot.commands.Auto_Adjust.AdjustToPose;
import frc.robot.commands.CollectGamePiece;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.JiggleCoral;
import frc.robot.commands.ScoreGamePiece;
import frc.robot.data.BranchLocation;
import frc.robot.data.DesiredReefPosition;
import frc.robot.data.ReefMap;
import frc.robot.generated.TunerConstants;
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
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Goal;
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
  private final Superstructure superstructure;
  private final Coral coral;
  private final Algae algae;
  private final Vision vision;
  private final Climber climber;

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
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        windmill = new Windmill(new WindmillIOTalonFX());
        elevator = new Elevator(new ElevatorIOTalonFX());
        coral = new Coral(new CoralIOSparkMax());
        algae = new Algae(new AlgaeIOSparkMax());
        climber = new Climber(new ClimberIOSparkMax());
        superstructure = new Superstructure(elevator, windmill, coral, algae);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1),
                new VisionIOPhotonVision(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        windmill = new Windmill(new WindmillIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        coral = new Coral(new CoralIOSim());
        algae = new Algae(new AlgaeIOSim());
        superstructure = new Superstructure(elevator, windmill, coral, algae);
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
        superstructure = new Superstructure(elevator, windmill, coral, algae);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});

        break;
    }

    NamedCommands.registerCommand("Intake Coral", new InstantCommand(() -> coral.runVolts(6.5)));
    NamedCommands.registerCommand("Stop Coral", new InstantCommand(() -> coral.runVolts(0)));
    NamedCommands.registerCommand("Outtake Coral", new InstantCommand(() -> coral.runVolts(-3)));
    NamedCommands.registerCommand(
        "Stop Algae Inner", new InstantCommand(() -> algae.setVoltageHolding(0)));
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

    NamedCommands.registerCommand(
        "Stow Position", Commands.runOnce(() -> superstructure.setAutoGoalCommand(Goal.STOW)));
    NamedCommands.registerCommand(
        "Collect Coral Position", superstructure.setGoalCommand(Goal.COLLECT));
    NamedCommands.registerCommand(
        "Coral Mode",
        Commands.runOnce(
            () -> superstructure.setAutoGamepieceCommand(GamePiece.CORAL), superstructure));
    NamedCommands.registerCommand(
        "Algae Mode",
        Commands.runOnce(
            () -> superstructure.setAutoGamepieceCommand(GamePiece.ALGAE), superstructure));
    NamedCommands.registerCommand(
        "Score L1 Coral Position",
        Commands.runOnce(() -> superstructure.setAutoGoalCommand(Goal.L1), superstructure));

    NamedCommands.registerCommand(
        "Auto Align Center", driveToReef(() -> drive.getDesiredReefFace(), BranchLocation.CENTER));

    NamedCommands.registerCommand(
        "Auto Align Left", driveToReef(() -> drive.getDesiredReefFace(), BranchLocation.LEFT));

    NamedCommands.registerCommand(
        "Auto Align Right", driveToReef(() -> drive.getDesiredReefFace(), BranchLocation.RIGHT));

    NamedCommands.registerCommand("Auto Align Source", drive.driveToStation());

    // NamedCommands.registerCommand(
    //     "Stay At Previous Position",
    //     superstructure.setAutoGoalCommand(superstructure.getCurrentGoal()));
    // NamedCommands.registerCommand(
    //     "Score L2 Coral Position", superstructure.setGoalCommand(Goal.SCORE_L2_CORAL));
    // NamedCommands.registerCommand(
    //     "Score L3 Coral Position", superstructure.setGoalCommand(Goal.SCORE_L3_CORAL));
    NamedCommands.registerCommand(
        "L2 Position",
        Commands.runOnce(() -> superstructure.setGoalCommand(Goal.L2), superstructure));
    NamedCommands.registerCommand(
        "L3 Position",
        Commands.runOnce(() -> superstructure.setAutoGoalCommand(Goal.L3), superstructure));
    NamedCommands.registerCommand(
        "Barge Position",
        Commands.runOnce(() -> superstructure.setAutoGoalCommand(Goal.BARGE), superstructure));

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

    controller.back().onTrue(Commands.runOnce(() -> drive.resetGyro(), drive));

    controller
        .leftBumper()
        .whileTrue(driveToReef(() -> drive.getDesiredReefFace(), BranchLocation.LEFT));
    controller
        .rightBumper()
        .whileTrue(driveToReef(() -> drive.getDesiredReefFace(), BranchLocation.RIGHT));
    controller.a().whileTrue(driveToReef(() -> drive.getDesiredReefFace(), BranchLocation.CENTER));
    controller.x().whileTrue(drive.driveToStation());
    // controller.b().whileTrue(drive.driveToBarge());

    controller.pov(0).onTrue(new InstantCommand(() -> climber.setVoltage(10)));
    controller.pov(0).onFalse(new InstantCommand(() -> climber.setVoltage(0)));

    controller.pov(180).onTrue(new InstantCommand(() -> climber.setVoltage(-10)));
    controller.pov(180).onFalse(new InstantCommand(() -> climber.setVoltage(0)));

    atariButton9.onTrue(new InstantCommand(() -> drive.setDesiredReefFace(TargetReefFace.A)));
    atariButton10.onTrue(new InstantCommand(() -> drive.setDesiredReefFace(TargetReefFace.B)));

    atariButton11.onTrue(new InstantCommand(() -> drive.setDesiredReefFace(TargetReefFace.C)));
    atariButton12.onTrue(new InstantCommand(() -> drive.setDesiredReefFace(TargetReefFace.D)));
    atariButton14.onTrue(new InstantCommand(() -> drive.setDesiredReefFace(TargetReefFace.E)));
    atariButton15.onTrue(new InstantCommand(() -> drive.setDesiredReefFace(TargetReefFace.F)));

    // if (atariButton13.getAsBoolean()) {
    //   coral.setCurrentGamePiece(GamePiece.ALGAE);

    // } else {

    //   coral.setCurrentGamePiece(GamePiece.CORAL);
    // }
    atariButton13.onTrue(superstructure.setGamepieceCommand(GamePiece.ALGAE));
    atariButton13.onFalse(superstructure.setGamepieceCommand(GamePiece.CORAL));

    atariButton1.onTrue(superstructure.setGoalCommand(Goal.STOW));
    atariButton2.onTrue(superstructure.setGoalCommand(Goal.L1));
    atariButton3.onTrue(superstructure.setGoalCommand(Goal.L2));
    atariButton4.onTrue(superstructure.setGoalCommand(Goal.L3));
    atariButton5.onTrue(superstructure.setGoalCommand(Goal.BARGE));
    atariButton6.onTrue(superstructure.setGoalCommand(Goal.COLLECT));
    atariButton8.whileTrue(new CollectGamePiece(coral, algae, superstructure));
    atariButton8.whileFalse(
        new ConditionalCommand(
            new JiggleCoral(coral),
            new InstantCommand(() -> coral.stop()),
            () -> GamePiece.CORAL.equals(superstructure.getGamePiece())));
    atariButton7.whileTrue(new ScoreGamePiece(coral, algae, superstructure));

    // atariButton1.onTrue(
    //     new ConditionalCommand(
    //         superstructure.setGoalCommand(Goal.ALGAE_PROCESSOR_AND_PRESTAGE),
    //         superstructure.setGoalCommand(Goal.SCORE_L1_CORAL),
    //         () -> atariButton13.getAsBoolean()));
    // atariButton2.onTrue(superstructure.setGoalCommand(Goal.SCORE_L2_CORAL));
    // atariButton3.onTrue(superstructure.setGoalCommand(Goal.SCORE_L3_CORAL));
    // atariButton4.onTrue(
    //     new ConditionalCommand(
    //         superstructure.setGoalCommand(Goal.COLLECT_CORAL),
    //         superstructure.setGoalCommand(Goal.SCORE_ALGAE_BARGE),
    //         () -> atariButton13.getAsBoolean()));
    // // atariButton5.onTrue(new CollectFromStation(superstructure));
    // atariButton6.onTrue(
    //     new ConditionalCommand(
    //         superstructure.setGoalCommand(Goal.STOW),
    //         superstructure.setGoalCommand(Goal.STOW),
    //         () -> atariButton13.getAsBoolean()));

    // controller
    //     .y()
    //     .onTrue(
    //         new ConditionalCommand(
    //             superstructure.setGoalCommand(Goal.SCORE_L3_CORAL),
    //             superstructure.setGoalCommand(Goal.SCORE_ALGAE_BARGE),
    //             controller.b()));
    // controller
    //     .a()
    //     .onTrue(
    //         new ConditionalCommand(
    //             superstructure.setGoalCommand(Goal.SCORE_L1_CORAL),
    //             superstructure.setGoalCommand(Goal.ALGAE_PROCESSOR_AND_PRESTAGE),
    //             controller.b()));

    // controller
    //     .x()
    //     .onTrue(
    //         new ConditionalCommand(
    //             superstructure.setGoalCommand(Goal.COLLECT_CORAL),
    //             superstructure.setGoalCommand(Goal.GRAB_L3_ALGAE),
    //             controller.b()));

    // controller.leftBumper().whileTrue(new CollectGamePiece(coral, algae));
    // controller.rightBumper().whileTrue(new ScoreGamePiece(coral, algae, superstructure));

    //  controller.a().onTrue(superstructure.setGoalCommand(Goal.STOW));
    // controller.x().onTrue(superstructure.setGoalCommand(Goal.SCORE_L1_CORAL));
    // controller.y().onTrue(superstructure.setGoalCommand(Goal.SCORE_L2_CORAL));

    // controller.y().onTrue(superstructure.setGoalCommand(Goal.SCORE_L3_CORAL));

    // controller.a().onTrue(new InstantCommand(() -> coral.runVolts(-6)));
    // controller.a().onFalse(new InstantCommand(() -> coral.runVolts(-0.25)));
    // controller.a().onFalse(new InstantCommand(() -> coral.runVolts(0)));

    // controller.x().onTrue(Commands.run(() -> windmill.setTargetPositionDegrees(-75), windmill));
    // controller.x().onTrue(new InstantCommand(() -> coral.runVolts(6)));

    // controller.x().onFalse(new InstantCommand(() -> coral.runVolts(0)));
    // controller.x().onTrue(Commands.run(() -> elevator.setTargetHeightInches(4), elevator));
    // controller.b().onTrue(Commands.run(() -> windmill.setTargetPositionDegrees(5), windmill));
    // controller.b().onTrue(Commands.run(() -> elevator.setTargetHeightInches(0.25), elevator));

    // controller.leftBumper().onTrue(new InstantCommand(() -> coral.runVolts(8)));
    // controller.leftBumper().onFalse(new InstantCommand(() -> coral.runVolts(0)));

    // controller.rightBumper().onTrue(new InstantCommand(() -> windmill.setVoltage(-3)));
    // controller.rightBumper().onFalse(new InstantCommand(() -> windmill.setVoltage(0)));

    // tuning/manual controls
    controller.b().onTrue(new InstantCommand(() -> windmill.setVoltage(-3)));
    controller.b().onFalse(new InstantCommand(() -> windmill.setVoltage(0)));

    controller.x().onTrue(new InstantCommand(() -> windmill.setVoltage(3)));
    controller.x().onFalse(new InstantCommand(() -> windmill.setVoltage(0)));

    controller.y().onTrue(new InstantCommand(() -> elevator.setVoltage(3)));
    controller.y().onFalse(new InstantCommand(() -> elevator.setVoltage(0.35)));

    controller.a().onTrue(new InstantCommand(() -> elevator.setVoltage(-3)));
    controller.a().onFalse(new InstantCommand(() -> elevator.setVoltage(0.35)));

    // controller.leftBumper().onTrue(new InstantCommand(() -> coral.runVolts(6)));
    // controller.leftBumper().onFalse(new InstantCommand(() -> coral.runVolts(0)));

    // controller.rightBumper().onTrue(new InstantCommand(() -> coral.runVolts(-5)));
    // controller.rightBumper().onFalse(new InstantCommand(() -> coral.runVolts(0)));

    // controller.leftBumper().onTrue(new InstantCommand(() -> algae.setVoltageLauncher(-9)));
    // controller.leftBumper().onTrue(new InstantCommand(() -> algae.setVoltageHolding(9)));
    // controller.leftBumper().onFalse(new InstantCommand(() -> algae.setVoltageLauncher(0)));
    // controller.leftBumper().onFalse(new InstantCommand(() -> algae.setVoltageHolding(0)));

    // controller.rightStick().onTrue(new InstantCommand(() -> algae.setVoltageLauncher(12)));
    // controller.rightBumper().onTrue(new InstantCommand(() -> algae.setVoltageHolding(-6)));
    // controller.rightStick().onFalse(new InstantCommand(() -> algae.setVoltageLauncher(0)));
    // controller.rightBumper().onFalse(new InstantCommand(() -> algae.setVoltageHolding(0)));

    controller.start().onTrue(new InstantCommand(() -> coral.toggleSolenoid()));
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
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          if (isFlipped) {
            // return AutoBuilder.pathfindToPose(
            //     goalPose
            //         .transformBy(robotTransform)
            //         .transformBy(desiredGamepieceTransform)
            //         .rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg),
            //     constraints);
            drive.setPose(vision.getReefCameraPose());
            return new AdjustToPose(
                goalPose
                    .transformBy(Constants.ROBOT_TRANSFORM)
                    .transformBy(desiredGamepieceTransform)
                    .rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg),
                drive);
          } else {
            return // AutoBuilder.pathfindToPose(goalPose.transformBy(robotTransform), constraints);
            new AdjustToPose(goalPose.transformBy(Constants.ROBOT_TRANSFORM), drive);
          }
        },
        Set.of(drive));
  }
}
