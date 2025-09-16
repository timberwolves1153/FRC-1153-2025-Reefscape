package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Superstructure;

public class SwapPiece extends Command {
    private Superstructure superstructure;

    public SwapPiece(Superstructure superstructure) {
        this.superstructure = superstructure;

        addRequirements(superstructure);
    }

    @Override
    public void execute() {
        if (superstructure.getGamePiece() == GamePiece.CORAL) {
            superstructure.setGamepieceCommand(GamePiece.ALGAE);
        } else {
            superstructure.setGamepieceCommand(GamePiece.CORAL);
        }
    }
}