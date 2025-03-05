package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Manipulator.Algae;
import frc.robot.subsystems.Manipulator.Coral;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Goal;

public class ScoreGamePiece extends Command {

  private Coral coral;
  private Algae algae;
  private Superstructure superstructure;
  private Timer timer;

  public ScoreGamePiece(Coral coral, Algae algae, Superstructure superstructure) {
    this.coral = coral;
    this.algae = algae;
    this.superstructure = superstructure;
    timer = new Timer();
    timer.reset();
    addRequirements(coral, algae);
  }

  @Override
  public void initialize() {
    timer.reset();
  }

  @Override
  public void execute() {

    timer.start();
    Goal currentGoal = superstructure.getCurrentGoal();
    GamePiece currentGamePiece = superstructure.getGamePiece();

    if (currentGamePiece.equals(GamePiece.CORAL)) {
      if (currentGoal.equals(Goal.L1)) {
        coral.runVolts();

      } else if (currentGoal.equals(Goal.L2)) {
        coral.runVolts(6);

      } else if (currentGoal.equals(Goal.L3)) {
        coral.runVolts(6);
      } else if (currentGoal.equals(Goal.BARGE)) {
        coral.runVolts(5);
      } else if (currentGoal.equals(Goal.COLLECT)) {
        coral.runVolts(4);
      }
    } else if (currentGamePiece.equals(GamePiece.ALGAE)) {
      if (currentGoal.equals(Goal.L1)) {
        algae.setVoltageLauncher(4);
        algae.setVoltageHolding(4);
      } else if ((currentGoal.equals(Goal.BARGE))) {

        algae.setVoltageLauncher(12);

        if (algae.inputs.outerAppliedVolts > 11.95
            && timer.get()
                > 1.25 /* isRobotAtDesiredPose, isLauncherReady, isWindmillReady, isElevatorReady */) {
          algae.setVoltageHolding(6);
        }
      }
    } else {
      algae.setVoltageHolding(0);
      algae.setVoltageLauncher(0);
      coral.runVolts(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.reset();
    algae.setVoltageHolding(0);
    algae.setVoltageLauncher(0);
    coral.runVolts(0);
  }
}
