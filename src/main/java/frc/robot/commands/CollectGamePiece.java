package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Manipulator.Algae;
import frc.robot.subsystems.Manipulator.Coral;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Goal;
import frc.robot.subsystems.toroSticks.GroundAlgae;

public class CollectGamePiece extends Command {

  private Coral coral;
  private Algae algae;
  private GroundAlgae groundAlgae;
  private Superstructure superstructure;
  private double coralHoldingVoltage;

  public CollectGamePiece(
      Coral coral, Algae algae, GroundAlgae groundAlgae, Superstructure superstructure) {
    this.coral = coral;
    this.algae = algae;
    this.groundAlgae = groundAlgae;
    this.superstructure = superstructure;
    coralHoldingVoltage = 0.1;

    addRequirements(coral, algae, groundAlgae);
  }

  @Override
  public void execute() {
    GamePiece selectedPiece = superstructure.getGamePiece();
    Goal currentGoal = superstructure.getCurrentGoal();

    if (GamePiece.CORAL.equals(selectedPiece)) {
      algae.stopHolding();
      algae.stopLauncher();
      groundAlgae.stow();
      groundAlgae.stopRollers();
      if (currentGoal.equals(Goal.L2) || currentGoal.equals(Goal.L3)) {
        coral.runVolts(2);
      } else {
        coral.runVolts(-6);
      }

    } else if (GamePiece.ALGAE.equals(selectedPiece)) {
      if (currentGoal.equals(Goal.COLLECT)) {
        coral.runVolts(-6);
      } else if (currentGoal.equals(Goal.L1)) {
        coral.runVolts(coralHoldingVoltage);
        algae.setVoltageHolding(6);
        algae.setVoltageLauncher(-6);
        groundAlgae.stow();
        groundAlgae.stopRollers();
      } else if (currentGoal.equals(Goal.GROUND)) {
        coral.runVolts(coralHoldingVoltage);
        algae.setVoltageHolding(6);
        algae.setVoltageLauncher(-6);
        groundAlgae.deploy();
        groundAlgae.intake();

      } else {

        coral.runVolts(coralHoldingVoltage);
        algae.setVoltageHolding(6);
        algae.setVoltageLauncher(-6);
        groundAlgae.stow();
        groundAlgae.stopRollers();
      }
    } else {
      coral.stop();
      algae.stopHolding();
      algae.stopLauncher();
      groundAlgae.stow();
      groundAlgae.stopRollers();
    }
  }

  @Override
  public void end(boolean interrupted) {
    GamePiece currPiece = superstructure.getGamePiece();
    if (GamePiece.CORAL.equals(currPiece)) {
      coral.jiggle();
      // coral.runVolts(coralHoldingVoltage);
    } else if (GamePiece.ALGAE.equals(currPiece)) {
      algae.setVoltageHolding(0);
      algae.setVoltageLauncher(0);
      groundAlgae.stopRollers();
      groundAlgae.stow();
    } else {
      // something has gone wrong, just dont run the collector
      algae.setVoltageHolding(0);
      algae.setVoltageLauncher(0);
      coral.runVolts(0);
      groundAlgae.stopRollers();
      groundAlgae.stow();
    }
  }
}
