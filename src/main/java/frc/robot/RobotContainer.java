// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Manipulator.Manipulator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...



  private final Joystick operator = new Joystick(1);

  private final JoystickButton opA = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton opB = new JoystickButton(operator, XboxController.Button.kB.value);

  private final Manipulator manipulator = new Manipulator();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings


    NamedCommands.registerCommand("Intake Manipulator", new InstantCommand(() -> manipulator.intakeCoral(), manipulator));
    NamedCommands.registerCommand("Outtake Manipulator", new InstantCommand(() -> manipulator.outtakeCoral(), manipulator));
    NamedCommands.registerCommand("Stop Manipulator", new InstantCommand(() -> manipulator.stopCoral(), manipulator));
    
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  
    opA.onTrue(new InstantCommand(() -> manipulator.intakeCoral(), manipulator));
    opA.onFalse(new InstantCommand(() -> manipulator.stopCoral(), manipulator));

    opB.onTrue(new InstantCommand(() -> manipulator.outtakeCoral(), manipulator));
    opB.onFalse(new InstantCommand(() -> manipulator.stopCoral(), manipulator));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  }


public Joystick getOperatorController(){
  return operator;
}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}