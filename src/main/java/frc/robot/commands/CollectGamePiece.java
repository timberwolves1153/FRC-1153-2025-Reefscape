package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Manipulator.Algae;
import frc.robot.subsystems.Manipulator.Coral;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Goal;

public class CollectGamePiece extends Command {

  private Coral coral;
  private Algae algae;
  private Superstructure superstructure;

  public CollectGamePiece(Coral coral, Algae algae, Superstructure superstructure) {
    this.coral = coral;
    this.algae = algae;
    this.superstructure = superstructure;

    addRequirements(coral, algae);
  }

  @Override
  public void execute() {
    GamePiece selectedPiece = superstructure.getGamePiece();
    Goal currentGoal = superstructure.getCurrentGoal();

    if (GamePiece.CORAL.equals(selectedPiece)) {
      if (currentGoal.equals(Goal.L2) || currentGoal.equals(Goal.L3)) {
        coral.runVolts(-2);
      } else {
        coral.runVolts(6);
      }

    } else if (GamePiece.ALGAE.equals(selectedPiece)) {
      algae.setVoltageHolding(6);
      algae.setVoltageLauncher(-6);
    } else {
      coral.stop();
      algae.stopHolding();
      algae.stopLauncher();
    }
  }

  @Override
  public void end(boolean interrupted) {
    GamePiece currPiece = superstructure.getGamePiece();
    if (GamePiece.CORAL.equals(currPiece)) {
      coral.runVolts(0);
    } else if (GamePiece.ALGAE.equals(currPiece)) {
      algae.setVoltageHolding(0);
      algae.setVoltageLauncher(0);
    } else {
      // something has gone wrong, just dont run the collector
      algae.setVoltageHolding(0);
      algae.setVoltageLauncher(0);
      coral.runVolts(0);
    }
  }
}
