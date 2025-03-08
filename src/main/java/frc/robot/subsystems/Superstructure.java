package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Manipulator.Algae;
import frc.robot.subsystems.Manipulator.Coral;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.windmill.Windmill;
import frc.robot.subsystems.windmill.Windmill.WindmillGoal;

public class Superstructure extends SubsystemBase {

  public enum Goal {
    STOW,
    COLLECT,
    // PRESTAGE_ALGAE,
    L1,
    L2,
    L3,
    // PROCESSOR_AND_PRESTAGE,
    BARGE,
    STAY_STILL;
  }

  private Goal desiredGoal = Goal.STOW;
  private GamePiece currentGamePiece = GamePiece.CORAL;

  private Elevator elevator;
  private Windmill windmill;
  private Coral coralManip;
  private Algae algaeManip;
  private Timer goalTimer = new Timer();

  public Superstructure(Elevator elevator, Windmill windmill, Coral coralManip, Algae algaeManip) {
    this.elevator = elevator;
    this.windmill = windmill;
    this.coralManip = coralManip;
    this.algaeManip = algaeManip;
  }

  private void setGoal(Goal goal) {
    if (desiredGoal == goal) {
      return; // The new goal is already our goal, do nothing
    }
    desiredGoal = goal; // Update the desired goal to be the new goal
  }

  public Goal getCurrentGoal() {
    return desiredGoal;
  }

  public Command setGoalCommand(Goal goal) {
    return startEnd(() -> setGoal(goal), () -> setGoal(getCurrentGoal()))
        .withName("Superstructure " + goal);
  }

  private void setGamepiece(GamePiece gamePiece) {
    if (currentGamePiece == gamePiece) {
      return; // The new gamepiece is already our state, do nothing
    }
    currentGamePiece = gamePiece; // Update the desired goal to be the new goal
  }

  public GamePiece getGamePiece() {
    return currentGamePiece;
  }

  public Command setGamepieceCommand(GamePiece gamePiece) {
    return startEnd(() -> setGamepiece(gamePiece), () -> setGamepiece(getGamePiece()))
        .withName("Superstructure Gamepiece" + gamePiece);
  }

  public void actuateCoralWhenAtPosition(Value pistonPosition, WindmillGoal goal) {
    if (windmill.isAtGoal(goal)) {
      coralManip.setSolenoidState(pistonPosition);
    }
  }

  public void rotateWindmillWhenAtPosition(Value pistonPosition, WindmillGoal goal) {
    if (coralManip.isAtGoal(pistonPosition)) {
      windmill.setTargetPosition(goal);
    }
  }

  @Override
  public void periodic() {

    if (DriverStation.isDisabled()) {
      setDefaultCommand(setGoalCommand(desiredGoal));
    }

    switch (desiredGoal) {
      case STOW -> {
        coralManip.setSolenoidState(Value.kForward);
        elevator.setTargetHeight(ElevatorGoal.STOW);
        windmill.setTargetPosition(WindmillGoal.STOW);
        SmartDashboard.putBoolean("Is At Goal", windmill.isAtGoal(WindmillGoal.STOW));
        // coralManip.runVolts(0.25);
        // algaeManip.setVoltageLauncher(0);

        // windmill go to this angle
        // coral retract
        break;
      }
      case COLLECT -> {
        elevator.setTargetHeight(ElevatorGoal.COLLECT_CORAL);
        // coralManip.setSolenoidState(Value.kForward);
        windmill.setTargetPosition(WindmillGoal.COLLECT_CORAL);
        break;
      }
      case L1 -> {
        if (GamePiece.CORAL.equals(getGamePiece())) {
          elevator.setTargetHeight(ElevatorGoal.L1_CORAL);
          windmill.setTargetPosition(WindmillGoal.L1_CORAL);
          SmartDashboard.putBoolean("Is At Goal", windmill.isAtGoal(WindmillGoal.L1_CORAL));
          actuateCoralWhenAtPosition(Value.kForward, WindmillGoal.L1_CORAL);
        } else if ((GamePiece.ALGAE.equals(getGamePiece()))) {
          elevator.setTargetHeight(ElevatorGoal.ALGAE_PROCESSOR_AND_PRESTAGE);
          coralManip.setSolenoidState(Value.kForward);
          // windmill.setTargetPosition(WindmillGoal.ALGAE_PROCESSOR_AND_PRESTAGE);
          windmill.setTargetPosition(WindmillGoal.ALGAE_PROCESSOR_AND_PRESTAGE);
        } else { // default is coral
          elevator.setTargetHeight(ElevatorGoal.L1_CORAL);
          windmill.setTargetPosition(WindmillGoal.L1_CORAL);
          SmartDashboard.putBoolean("Is At Goal", windmill.isAtGoal(WindmillGoal.L1_CORAL));
          actuateCoralWhenAtPosition(Value.kForward, WindmillGoal.L1_CORAL);
        }
        break;
      }
      case L2 -> {
        if (GamePiece.CORAL.equals(getGamePiece())) {
          elevator.setTargetHeight(ElevatorGoal.L2_CORAL);
          windmill.setTargetPosition(WindmillGoal.L2_CORAL);
          actuateCoralWhenAtPosition(Value.kReverse, WindmillGoal.L2_CORAL);
        } else if ((GamePiece.ALGAE.equals(getGamePiece()))) {
          elevator.setTargetHeight(ElevatorGoal.L2_ALGAE);
          // windmill.setTargetPosition(WindmillGoal.L2_ALGAE);
          coralManip.setSolenoidState(Value.kForward);
          windmill.setTargetPosition(WindmillGoal.L2_ALGAE);
        } else {
          elevator.setTargetHeight(ElevatorGoal.L2_CORAL);
          windmill.setTargetPosition(WindmillGoal.L2_CORAL);
        }
        break;
      }
      case L3 -> {
        if (GamePiece.CORAL.equals(getGamePiece())) {
          elevator.setTargetHeight(ElevatorGoal.L3_CORAL);
          windmill.setTargetPosition(WindmillGoal.L3_CORAL);
          actuateCoralWhenAtPosition(Value.kReverse, WindmillGoal.L3_CORAL);

        } else if ((GamePiece.ALGAE.equals(getGamePiece()))) {
          elevator.setTargetHeight(ElevatorGoal.L3_ALGAE);
          coralManip.setSolenoidState(Value.kForward);
          windmill.setTargetPosition(WindmillGoal.L3_ALGAE);

        } else {
          elevator.setTargetHeight(ElevatorGoal.L3_CORAL);
          windmill.setTargetPosition(WindmillGoal.L3_CORAL);
          // actuateCoralWhenAtPosition(Value.kReverse, WindmillGoal.L3_CORAL);
        }
        break;
      }
      case BARGE -> {
        if (GamePiece.CORAL.equals(getGamePiece())) {
          elevator.setTargetHeight(ElevatorGoal.L4_CORAL);
          windmill.setTargetPosition(WindmillGoal.L4_CORAL);
        } else if (GamePiece.CORAL.equals(getGamePiece())) {
          elevator.setTargetHeight(ElevatorGoal.ALGAE_BARGE);
          windmill.setTargetPosition(WindmillGoal.ALGAE_BARGE);
        } else {
          elevator.setTargetHeight(ElevatorGoal.L4_CORAL);
          windmill.setTargetPosition(WindmillGoal.L4_CORAL);
        }

        break;
      }

      case STAY_STILL -> {
        elevator.setTargetHeightInches(elevator.elevatorInputs.heightInches);
        windmill.setTargetPositionDegrees(
            Units.rotationsToDegrees(windmill.windmillInputs.rotations));
        coralManip.setSolenoidState(coralManip.getSolenoidState());

        break;
      }
      default -> {
        elevator.setTargetHeight(ElevatorGoal.STOW);
        windmill.setTargetPosition(WindmillGoal.STOW);
        coralManip.setSolenoidState(Value.kForward);

        break;
      }
    }
    SmartDashboard.putString("Goal", desiredGoal.toString());
    SmartDashboard.putString("Gamepiece", getGamePiece().toString());
  }
}
