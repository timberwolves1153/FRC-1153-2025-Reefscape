package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Goal;
import frc.robot.subsystems.Manipulator.Algae;
import frc.robot.subsystems.Manipulator.Coral;

public class ScoreGamePiece extends Command {

    private Coral coral;
    private Algae algae;
    private Superstructure superstructure;


    public ScoreGamePiece(Coral coral, Algae algae, Superstructure superstructure) {
        this.coral = coral;
        this.algae = algae;
        this.superstructure = superstructure;

        addRequirements(coral, algae);
    }

    @Override
    public void execute() {
        Goal currentGoal = superstructure.getCurrentGoal();

        if (currentGoal.equals(Goal.SCORE_L1_CORAL)) {
            coral.runVolts(-6);

        } else if (currentGoal.equals(Goal.SCORE_L2_CORAL)) {
            coral.runVolts(6);

        } else if (currentGoal.equals(Goal.SCORE_L3_CORAL)) {
            coral.runVolts(6);

        } else if (currentGoal.equals(Goal.SCORE_ALGAE_PROCESSOR)) {
            algae.setVoltageLauncher(-6);
            algae.setVoltageHolding(-6);

        } else if ((currentGoal.equals(Goal.SCORE_ALGAE_BARGE))) {
            algae.setVoltageLauncher(-12);
            if (algae.inputs.outerAppliedVolts > 11.7 /* isRobotAtDesiredPose, isLauncherReady, isWindmillReady, isElevatorReady */) {
                algae.setVoltageHolding(-6);
            }

        }

       
    }
}
