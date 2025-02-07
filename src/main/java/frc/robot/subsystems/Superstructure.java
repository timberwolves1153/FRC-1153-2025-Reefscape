package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.windmill.Windmill;
import frc.robot.subsystems.windmill.Windmill.WindmillGoal;

public class Superstructure extends SubsystemBase {

  public enum Goal {
    STOW,
    COLLECT_CORAL,
    SCORE_L1_CORAL,
    SCORE_L2_CORAL,
    GRAB_L2_ALGAE,
    SCORE_L3_CORAL,
    GRAB_L3_ALGAE,
    SCORE_ALGAE_PROCESSOR,
    SCORE_ALGAE_BARGE;
  }

  private Goal desiredGoal = Goal.STOW;

  private Elevator elevator;
  private Windmill windmill;

  private Timer goalTimer = new Timer();

  public Superstructure(Elevator elevator, Windmill windmill) {
    this.elevator = elevator;
    this.windmill = windmill;
  }

  private void setGoal(Goal goal) {
    if (desiredGoal == goal) {
      return; // The new goal is already our goal, do nothing
    }
    desiredGoal = goal; // Update the desired goal to be the new goal
  }

  public Command setGoalCommand(Goal goal) {
    return startEnd(() -> setGoal(goal), () -> setGoal(Goal.STOW))
        .withName("Superstructure " + goal);
  }

  @Override
  public void periodic() {

    if (DriverStation.isDisabled()) {
      setDefaultCommand(setGoalCommand(Goal.STOW));
    }

    switch (desiredGoal) {
      case STOW -> {
        elevator.setTargetHeight(ElevatorGoal.STOW);
        windmill.setTargetPosition(WindmillGoal.STOW);
        // windmill go to this angle
        // coral retract
        break;
      }
      case COLLECT_CORAL -> {
        elevator.setTargetHeight(ElevatorGoal.L1_CORAL);
        windmill.setTargetPosition(WindmillGoal.COLLECT_CORAL);
        // windmill to this angle
        // piston on coral retracted
        break;
      }
      case SCORE_L1_CORAL -> {
        elevator.setTargetHeight(ElevatorGoal.L1_CORAL);
        windmill.setTargetPosition(WindmillGoal.L1_CORAL);
        break;
      }
      case SCORE_L2_CORAL -> {
        elevator.setTargetHeight(ElevatorGoal.L2_CORAL);
        break;
      }
      default -> {
        elevator.setTargetHeight(ElevatorGoal.STOW);
        windmill.setTargetPosition(WindmillGoal.STOW);
      }
    }
    SmartDashboard.putString("Goal", desiredGoal.toString());
  }
}
