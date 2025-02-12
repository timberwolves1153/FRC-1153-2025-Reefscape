package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Goal;
import frc.robot.subsystems.Manipulator.Coral;

public class GoToL4OrBarge extends Command{
       private Superstructure superstructure;
    private Coral coral;
    
    public GoToL4OrBarge(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void execute() {
        GamePiece selectedGamePiece = coral.getCurrentGamePiece();
        if (selectedGamePiece.equals(GamePiece.CORAL)) {
            superstructure.setGoalCommand(Goal.STOW);
        } else if (selectedGamePiece.equals(GamePiece.ALGAE)) {
            superstructure.setGoalCommand(Goal.SCORE_ALGAE_BARGE);
        } else {
            superstructure.setGoalCommand(Goal.STOW);
        }
    }
}
