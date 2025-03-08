package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Manipulator.Coral;

public class JiggleCoral extends SequentialCommandGroup {

  private Coral coral;

  public JiggleCoral(Coral coral) {

    this.coral = coral;
    addRequirements(coral);

    addCommands(
        Commands.runOnce(() -> coral.runVolts(-3)),
        new WaitCommand(0.1),
        Commands.runOnce(() -> coral.runVolts(3)),
        new WaitCommand(0.1),
        Commands.runOnce(() -> coral.runVolts(-3)),
        new WaitCommand(0.1),
        // Commands.runOnce(() -> coral.runVolts(3)),
        // new WaitCommand(0.1),
        Commands.runOnce(() -> coral.runVolts(0.1)));
  }
}
