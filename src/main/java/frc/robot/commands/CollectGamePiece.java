package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Manipulator.Algae;
import frc.robot.subsystems.Manipulator.Coral;

public class CollectGamePiece extends Command {

  private Coral coral;
  private Algae algae;

  public CollectGamePiece(Coral coral, Algae algae) {
    this.coral = coral;
    this.algae = algae;

    addRequirements(coral, algae);
  }

  @Override
  public void execute() {
    GamePiece selectedPiece = coral.getCurrentGamePiece();

    if (GamePiece.CORAL.equals(selectedPiece)) {
      coral.runVolts(6);
    } else if (GamePiece.ALGAE.equals(selectedPiece)) {
      algae.setVoltageHolding(6);
      algae.setVoltageLauncher(6);
    } else {
        coral.stop();
        algae.stopHolding();
        algae.stopLauncher();
    }
  }
}
