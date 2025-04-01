package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamePiece;
import frc.robot.FieldConstants;
import frc.robot.Interpolation.InterpolatingDouble;
import frc.robot.Interpolation.WindmillTable;
import frc.robot.subsystems.Manipulator.Algae;
import frc.robot.subsystems.Manipulator.Coral;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Goal;
import frc.robot.subsystems.drive.Drive;

public class ScoreGamePiece extends Command {

  private Coral coral;
  private Algae algae;
  private Drive drive;
  private Superstructure superstructure;
  private Timer timer;
  private WindmillTable launcherMap;

  public ScoreGamePiece(Coral coral, Algae algae, Drive drive, Superstructure superstructure) {
    this.coral = coral;
    this.algae = algae;
    this.superstructure = superstructure;
    this.drive = drive;
    timer = new Timer();
    timer.reset();
    launcherMap = new WindmillTable();
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
        coral.runVolts(2);

      } else if (currentGoal.equals(Goal.L2)) {
        coral.runVolts(5);

      } else if (currentGoal.equals(Goal.L3)) {
        coral.runVolts(5);
      } else if (currentGoal.equals(Goal.BARGE)) {
        coral.runVolts(5);
      } else if (currentGoal.equals(Goal.COLLECT)) {
        coral.runVolts(4);
      }
    } else if (currentGamePiece.equals(GamePiece.ALGAE)) {
      if (currentGoal.equals(Goal.L1)) {

        algae.setVoltageLauncher(4);
        algae.setVoltageHolding(-4);
      } else if ((currentGoal.equals(Goal.BARGE))) {
        double shootingVolts =
            launcherMap.launcherMap.getInterpolated(
                    new InterpolatingDouble(FieldConstants.getNearestCage(drive.getPose())))
                .value;

        algae.setVoltageLauncher(shootingVolts);

        if (algae.inputs.outerAppliedVolts > (shootingVolts - 0.05)
        /* isRobotAtDesiredPose, isLauncherReady, isWindmillReady, isElevatorReady */ ) {
          algae.setVoltageHolding(-6);
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
