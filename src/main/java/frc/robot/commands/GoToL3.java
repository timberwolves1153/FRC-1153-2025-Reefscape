package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Manipulator.Coral;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Goal;

public class GoToL3 extends Command {
  private Superstructure superstructure;
  private Coral coral;

  public GoToL3(Superstructure superstructure, Coral coral) {
    this.superstructure = superstructure;
    this.coral = coral;

    addRequirements(superstructure, coral);
  }

  @Override
  public void execute() {
    GamePiece selectedGamePiece = coral.getCurrentGamePiece();
    if (selectedGamePiece.equals(GamePiece.CORAL)) {
      superstructure.setGoalCommand(Goal.SCORE_L3_CORAL);
    } else if (selectedGamePiece.equals(GamePiece.ALGAE)) {
      superstructure.setGoalCommand(Goal.GRAB_L3_ALGAE);
    } else {
      superstructure.setGoalCommand(Goal.STOW);
    }
  }
}
