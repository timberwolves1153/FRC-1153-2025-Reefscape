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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.data.BranchLocation;
import frc.robot.data.DesiredReefPosition;
import frc.robot.generated.TunerConstants;
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
import java.util.HashMap;
import java.util.Map;
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
  private final Elevator elevator;
  private final Windmill windmill;
  // private final Superstructure superstructure;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final Map<DesiredReefPosition, Pose2d> reefmap = buildReefMap();

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
        // superstructure = new Superstructure(elevator, windmill);
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
        // superstructure = new Superstructure(elevator, windmill);
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
        // superstructure = new Superstructure(elevator, windmill);
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

    // controller.x().onFalse(new InstantCommand(() -> elevator.holdTargetHeight(), elevator));

    //  controller.b().onTrue(Commands.run(() -> elevator.setTargetHeight(0.0), elevator));
    // controller.b().onFalse(new InstantCommand(() -> elevator.holdTargetHeight(), elevator));

    // operator.y().whileTrue(elevator.runCharacterizationQuasiForward());

    // operator.a().whileTrue(elevator.runCharacterizationQuasiReserve());

    // operator.b().whileTrue(elevator.runCharacterizationDynamReverse());

    // operator.x().whileTrue(elevator.runCharacterizationDynamForward());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private Map<DesiredReefPosition, Pose2d> buildReefMap() {

    FieldConstants.Reef reef = new FieldConstants.Reef();
    //       3 d
    //       __
    // 2 c  /  \ 4 e
    // 1 b  \__/ 5 f
    //       0 a
    Map<DesiredReefPosition, Pose2d> reefMap = new HashMap<>();

    DesiredReefPosition aLeft = new DesiredReefPosition(0, BranchLocation.LEFT);
    DesiredReefPosition aRight = new DesiredReefPosition(0, BranchLocation.RIGHT);
    DesiredReefPosition aCenter = new DesiredReefPosition(0, BranchLocation.CENTER);

    DesiredReefPosition bLeft = new DesiredReefPosition(1, BranchLocation.LEFT);
    DesiredReefPosition bRight = new DesiredReefPosition(1, BranchLocation.RIGHT);
    DesiredReefPosition bCenter = new DesiredReefPosition(1, BranchLocation.CENTER);

    DesiredReefPosition cLeft = new DesiredReefPosition(2, BranchLocation.LEFT);
    DesiredReefPosition cRight = new DesiredReefPosition(2, BranchLocation.RIGHT);
    DesiredReefPosition cCenter = new DesiredReefPosition(2, BranchLocation.CENTER);

    DesiredReefPosition dLeft = new DesiredReefPosition(3, BranchLocation.LEFT);
    DesiredReefPosition dRight = new DesiredReefPosition(3, BranchLocation.RIGHT);
    DesiredReefPosition dCenter = new DesiredReefPosition(3, BranchLocation.CENTER);

    DesiredReefPosition eLeft = new DesiredReefPosition(4, BranchLocation.LEFT);
    DesiredReefPosition eRight = new DesiredReefPosition(4, BranchLocation.RIGHT);
    DesiredReefPosition eCenter = new DesiredReefPosition(4, BranchLocation.CENTER);

    DesiredReefPosition fLeft = new DesiredReefPosition(5, BranchLocation.LEFT);
    DesiredReefPosition fRight = new DesiredReefPosition(5, BranchLocation.RIGHT);
    DesiredReefPosition fCenter = new DesiredReefPosition(5, BranchLocation.CENTER);

    // lower number always on right for field constants
    //       78
    //       __
    // 6 5  /  \ 9 10
    // 4 3  \__/ 11 12
    //       2 1
    reefMap.put(
        aLeft,
        FieldConstants.Reef.branchPositions.get(1).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(
        aRight,
        FieldConstants.Reef.branchPositions.get(0).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(aCenter, FieldConstants.Reef.centerFaces[0]);

    reefMap.put(
        bLeft,
        FieldConstants.Reef.branchPositions.get(3).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(
        bRight,
        FieldConstants.Reef.branchPositions.get(2).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(bCenter, FieldConstants.Reef.centerFaces[1]);

    reefMap.put(
        cLeft,
        FieldConstants.Reef.branchPositions.get(5).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(
        cRight,
        FieldConstants.Reef.branchPositions.get(4).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(cCenter, FieldConstants.Reef.centerFaces[2]);

    reefMap.put(
        dLeft,
        FieldConstants.Reef.branchPositions.get(6).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(
        dRight,
        FieldConstants.Reef.branchPositions.get(7).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(dCenter, FieldConstants.Reef.centerFaces[3]);

    reefMap.put(
        eLeft,
        FieldConstants.Reef.branchPositions.get(8).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(
        eRight,
        FieldConstants.Reef.branchPositions.get(9).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(eCenter, FieldConstants.Reef.centerFaces[4]);

    reefMap.put(
        fLeft,
        FieldConstants.Reef.branchPositions.get(10).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(
        fRight,
        FieldConstants.Reef.branchPositions.get(11).get(FieldConstants.ReefHeight.L2).toPose2d());
    reefMap.put(fCenter, FieldConstants.Reef.centerFaces[5]);

    Logger.recordOutput("aLeft", reefMap.get(aLeft));
    Logger.recordOutput("aRight", reefMap.get(aRight));
    Logger.recordOutput("aCenter", reefMap.get(aCenter));

    Logger.recordOutput("bLeft", reefMap.get(bLeft));
    Logger.recordOutput("bRight", reefMap.get(bRight));
    Logger.recordOutput("bCenter", reefMap.get(bCenter));

    Logger.recordOutput("cLeft", reefMap.get(cLeft));
    Logger.recordOutput("cRight", reefMap.get(cRight));
    Logger.recordOutput("cCenter", reefMap.get(cCenter));

    Logger.recordOutput("dLeft", reefMap.get(dLeft));
    Logger.recordOutput("dRight", reefMap.get(dRight));
    Logger.recordOutput("dCenter", reefMap.get(dCenter));

    Logger.recordOutput("eLeft", reefMap.get(eLeft));
    Logger.recordOutput("eRight", reefMap.get(eRight));
    Logger.recordOutput("eCenter", reefMap.get(eCenter));

    Logger.recordOutput("fLeft", reefMap.get(fLeft));
    Logger.recordOutput("fRight", reefMap.get(fRight));
    Logger.recordOutput("fCenter", reefMap.get(fCenter));

    return reefMap;
  }
}
