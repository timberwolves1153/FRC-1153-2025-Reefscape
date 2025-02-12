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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.GamePiece;
import frc.robot.commands.CollectGamePiece;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ScoreGamePiece;
import frc.robot.generated.TunerConstants;
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
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
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
  private final Windmill windmill;
  private final Elevator elevator;
  private final Superstructure superstructure;
  private final Coral coral;
  private final Algae algae;

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

        windmill = new Windmill(new WindmillIOTalonFX());
        elevator = new Elevator(new ElevatorIOTalonFX());
        coral = new Coral(new CoralIOSparkMax());
        algae = new Algae(new AlgaeIOSparkMax());
        superstructure = new Superstructure(elevator, windmill, coral, algae);
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
        superstructure = new Superstructure(elevator, windmill, coral, algae);
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

    // NamedCommands.registerCommand("Intake Coral", new InstantCommand(() -> coral.runVolts(6)));
    // NamedCommands.registerCommand(
    //     "Stop Coral Collector", new InstantCommand(() -> coral.runVolts(0)));
    // NamedCommands.registerCommand("Outtake Coral", new InstantCommand(() -> coral.runVolts(-6)));
    // NamedCommands.registerCommand(
    //     "Grab Algae Inner", new InstantCommand(() -> algae.setVoltageHolding(6)));
    // NamedCommands.registerCommand(
    //     "Grab Algae Outer", new InstantCommand(() -> algae.setVoltageHolding(6)));
    // NamedCommands.registerCommand(
    //     "Shoot Algae Inner", new InstantCommand(() -> algae.setVoltageHolding(-6)));
    // NamedCommands.registerCommand(
    //     "Shoot Algae Outer", new InstantCommand(() -> algae.setVoltageHolding(-12)));
    // NamedCommands.registerCommand(
    //     "Stop Algae Inner", new InstantCommand(() -> algae.setVoltageHolding(0)));
    // NamedCommands.registerCommand(
    //     "Stop Algae Outer", new InstantCommand(() -> algae.setVoltageHolding(0)));

    // NamedCommands.registerCommand("Stow Position", superstructure.setGoalCommand(Goal.STOW));
    // NamedCommands.registerCommand(
    //     "Collect Coral Position", superstructure.setGoalCommand(Goal.COLLECT_CORAL));
    // NamedCommands.registerCommand(
    //     "Score L1 Coral Position", superstructure.setGoalCommand(Goal.SCORE_L1_CORAL));
    // NamedCommands.registerCommand(
    //     "Score L2 Coral Position", superstructure.setGoalCommand(Goal.SCORE_L2_CORAL));
    // NamedCommands.registerCommand(
    //     "Score L3 Coral Position", superstructure.setGoalCommand(Goal.SCORE_L3_CORAL));
    // NamedCommands.registerCommand(
    //     "Grab L2 Algae Position", superstructure.setGoalCommand(Goal.GRAB_L2_ALGAE));
    // NamedCommands.registerCommand(
    //     "Grab L3 Algae Position", superstructure.setGoalCommand(Goal.GRAB_L3_ALGAE));
    // NamedCommands.registerCommand(
    //     "Shoot Algae Position", superstructure.setGoalCommand(Goal.SCORE_ALGAE_BARGE));

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
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    if (atariButton13.getAsBoolean()) {
      coral.setCurrentGamePiece(GamePiece.ALGAE);

    } else {

      coral.setCurrentGamePiece(GamePiece.CORAL);
    }

    if (coral.getCurrentGamePiece().equals(GamePiece.CORAL)) {
      atariButton1.onTrue(superstructure.setGoalCommand(Goal.SCORE_L1_CORAL));
      atariButton2.onTrue(superstructure.setGoalCommand(Goal.SCORE_L2_CORAL));
      // atariButton3.onTrue(superstructure.setGoalCommand(Goal.SCORE_L3_CORAL));
      atariButton6.onTrue(superstructure.setGoalCommand(Goal.STOW));
      atariButton5.onTrue(superstructure.setGoalCommand(Goal.COLLECT_CORAL));
      atariButton4.onTrue(superstructure.setGoalCommand(Goal.COLLECT_CORAL));

    } else {

      atariButton1.onTrue(superstructure.setGoalCommand(Goal.ALGAE_PROCESSOR_AND_PRESTAGE));
      atariButton2.onTrue(superstructure.setGoalCommand(Goal.GRAB_L2_ALGAE));
      atariButton3.onTrue(superstructure.setGoalCommand(Goal.GRAB_L3_ALGAE));
      atariButton6.onTrue(superstructure.setGoalCommand(Goal.STOW));
      atariButton5.onTrue(superstructure.setGoalCommand(Goal.ALGAE_PROCESSOR_AND_PRESTAGE));
      atariButton4.onTrue(superstructure.setGoalCommand(Goal.SCORE_ALGAE_BARGE));
    }

    atariButton3.onTrue(superstructure.setGoalCommand(Goal.SCORE_L3_CORAL));

    atariButton7.whileTrue(new CollectGamePiece(coral, algae));

    atariButton8.whileTrue(new ScoreGamePiece(coral, algae, superstructure));

    // controller.a().onTrue(superstructure.setGoalCommand(Goal.STOW));

    // controller.x().onTrue(superstructure.setGoalCommand(Goal.SCORE_L1_CORAL));
    // controller.y().onTrue(superstructure.setGoalCommand(Goal.SCORE_L2_CORAL));

    // controller.start().onTrue(superstructure.setGoalCommand(Goal.SCORE_L3_CORAL));

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
    // controller.b().onTrue(new InstantCommand(() -> windmill.setVoltage(-3)));
    // controller.b().onFalse(new InstantCommand(() -> windmill.setVoltage(0)));

    // controller.x().onTrue(new InstantCommand(() -> windmill.setVoltage(3)));
    // controller.x().onFalse(new InstantCommand(() -> windmill.setVoltage(0)));

    // controller.y().onTrue(new InstantCommand(() -> elevator.setVoltage(3)));
    // controller.y().onFalse(new InstantCommand(() -> elevator.setVoltage(0.25)));

    // controller.a().onTrue(new InstantCommand(() -> elevator.setVoltage(-3)));
    // controller.a().onFalse(new InstantCommand(() -> elevator.setVoltage(0.25)));

    // controller.leftBumper().onTrue(new InstantCommand(() -> coral.runVolts(6)));
    // controller.leftBumper().onFalse(new InstantCommand(() -> coral.runVolts(0)));

    // controller.rightBumper().onTrue(new InstantCommand(() -> coral.runVolts(-5)));
    // controller.rightBumper().onFalse(new InstantCommand(() -> coral.runVolts(0)));

    // controller.leftBumper().onTrue(new InstantCommand(() -> algae.setVoltageLauncher(-6)));
    // controller.leftBumper().onTrue(new InstantCommand(() -> algae.setVoltageHolding(-6)));
    // controller.leftBumper().onFalse(new InstantCommand(() -> algae.setVoltageLauncher(0)));
    // controller.leftBumper().onFalse(new InstantCommand(() -> algae.setVoltageHolding(0)));

    // controller.rightStick().onTrue(new InstantCommand(() -> algae.setVoltageLauncher(12)));
    // controller.rightBumper().onTrue(new InstantCommand(() -> algae.setVoltageHolding(6)));
    // controller.rightStick().onFalse(new InstantCommand(() -> algae.setVoltageLauncher(0)));
    // controller.rightBumper().onFalse(new InstantCommand(() -> algae.setVoltageHolding(0)));

    // controller.start().onTrue(new InstantCommand(() -> coral.toggleSolenoid()));

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
