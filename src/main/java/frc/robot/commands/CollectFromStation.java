package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Manipulator.Coral;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Goal;

public class CollectFromStation extends Command {
  private Superstructure superstructure;
  private Coral coral;

  public CollectFromStation(Superstructure superstructure) {
    this.superstructure = superstructure;
  }

  @Override
  public void execute() {
    GamePiece selectedGamePiece = coral.getCurrentGamePiece();
    if (selectedGamePiece.equals(GamePiece.CORAL)) {
      superstructure.setGoalCommand(Goal.COLLECT_CORAL);
    } else if (selectedGamePiece.equals(GamePiece.ALGAE)) {
      superstructure.setGoalCommand(Goal.ALGAE_PROCESSOR_AND_PRESTAGE);
    } else {
      superstructure.setGoalCommand(Goal.STOW);
    }
  }
}
