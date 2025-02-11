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

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.camera2Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera2;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.data.BranchLocation;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Manipulator.Algae;
import frc.robot.subsystems.Manipulator.AlgaeIOSparkMax;
import frc.robot.subsystems.Manipulator.Coral;
import frc.robot.subsystems.Manipulator.CoralIOSparkMax;
import frc.robot.subsystems.Superstructure;
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
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.windmill.Windmill;
import frc.robot.subsystems.windmill.WindmillIO;
import frc.robot.subsystems.windmill.WindmillIOSim;
import frc.robot.subsystems.windmill.WindmillIOTalonFX;
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
  private final Elevator elevator;
  private final Windmill windmill;
  private final Superstructure superstructure;
  private final Vision vision;
  private final Coral coral = new Coral(new CoralIOSparkMax());
  private final Algae algae = new Algae(new AlgaeIOSparkMax());

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private final Joystick atari = new Joystick(1);

  private final JoystickButton atariButton1 = new JoystickButton(atari, 1);
  private final JoystickButton atariButton2 = new JoystickButton(atari, 2);
  private final JoystickButton atariButton3 = new JoystickButton(atari, 3);
  private final JoystickButton atariButton4 = new JoystickButton(atari, 4);
  private final JoystickButton atariButton5 = new JoystickButton(atari, 5);
  private final JoystickButton atariButton6 = new JoystickButton(atari, 6);
  private final JoystickButton atariButton7 = new JoystickButton(atari, 7);
  private final JoystickButton atariButton8 = new JoystickButton(atari, 8);
  private final JoystickButton atariButton9 = new JoystickButton(atari, 9);
  private final JoystickButton atariButton10 = new JoystickButton(atari, 10);
  private final JoystickButton atariButton11 = new JoystickButton(atari, 11);
  private final JoystickButton atariButton12 = new JoystickButton(atari, 12);
  private final JoystickButton atariButton13 = new JoystickButton(atari, 13);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
        elevator = new Elevator(new ElevatorIOTalonFX());
        windmill = new Windmill(new WindmillIOTalonFX());
        superstructure = new Superstructure(elevator, windmill);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1),
                new VisionIOPhotonVision(camera2Name, robotToCamera2));
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
        elevator = new Elevator(new ElevatorIOSim());
        windmill = new Windmill(new WindmillIOSim());
        superstructure = new Superstructure(elevator, windmill);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose));
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
        elevator = new Elevator(new ElevatorIO() {});
        windmill = new Windmill(new WindmillIO() {});
        superstructure = new Superstructure(elevator, windmill);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // controller.start().whileTrue(drive.pathFindCommand());

    // atariButton1.whileTrue(drive.pathFindCommand(() -> 1, BranchLocation.RIGHT));
    // atariButton2.whileTrue(drive.pathFindCommand(() -> 2, BranchLocation.LEFT));
    // atariButton3.whileTrue(drive.pathFindCommand(() -> 3, BranchLocation.LEFT));
    atariButton1.onTrue(new InstantCommand(() -> drive.setDesiredReefFace(TargetReefFace.A)));
    atariButton2.onTrue(new InstantCommand(() -> drive.setDesiredReefFace(TargetReefFace.B)));
    controller
        .leftBumper()
        .whileTrue(drive.pathFindCommand(() -> drive.getDesiredReefFace(), BranchLocation.LEFT));
    // controller
    //     .rightBumper()
    //     .whileTrue(
    //         drive.pathFindCommand(() -> drive.getDesiredReefFace().index, BranchLocation.RIGHT));
    // controller
    //     .start()
    //     .whileTrue(
    //         drive.pathFindCommand(() -> drive.getDesiredReefFace().index,
    // BranchLocation.CENTER));
    // controller.x().onFalse(new InstantCommand(() -> elevator.holdTargetHeight(), elevator));
    // Windmill controls

    // elevator controls
    // controller.y().onTrue(new InstantCommand(() -> elevator.setVoltage(3), elevator));
    // controller.y().onFalse(new InstantCommand(() -> elevator.setVoltage(0.25), elevator));

    // controller.a().onTrue(new InstantCommand(() -> elevator.setVoltage(-2), elevator));
    // controller.a().onFalse(new InstantCommand(() -> elevator.setVoltage(0.25), elevator));

    // controller.x().onTrue(Commands.run(() -> elevator.setTargetHeightInches(10.0), elevator));
    // controller.b().onTrue(superstructure.setGoalCommand(Goal.SCORE_L1_CORAL));
    // controller.x().onFalse(new InstantCommand(() -> elevator.holdTargetHeight(), elevator));

    // operator.leftBumper().onTrue(new InstantCommand(() -> coral.runVolts(4), coral));
    // operator.leftBumper().onFalse(new InstantCommand(() -> coral.stop(), coral));

    // // operator.rightBumper().onTrue(new InstantCommand(() -> coral.runVolts(-6), coral));
    // operator.rightBumper().onFalse(new InstantCommand(() -> coral.stop(), coral));

    // // operator.a().onTrue(new InstantCommand(() -> coral.setSolenoid(), coral));

    // // operator.leftStick().onTrue(new InstantCommand(() -> algae.runVoltsOuter(4), algae));
    // operator.leftStick().onFalse(new InstantCommand(() -> algae.stopOuter(), algae));

    // // operator.rightStick().onTrue(new InstantCommand(() -> algae.runVoltsOuter(-4), algae));
    // operator.rightStick().onFalse(new InstantCommand(() -> algae.stopOuter(), algae));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
