package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Manipulator.Algae;
import frc.robot.subsystems.Manipulator.Coral;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.windmill.Windmill;
import frc.robot.subsystems.windmill.Windmill.WindmillGoal;

public class Superstructure extends SubsystemBase {

  public enum Goal {
    STOW,
    COLLECT_CORAL,
    // PRESTAGE_ALGAE,
    SCORE_L1_CORAL,
    SCORE_L2_CORAL,
    GRAB_L2_ALGAE,
    SCORE_L3_CORAL,
    GRAB_L3_ALGAE,
    ALGAE_PROCESSOR_AND_PRESTAGE,
    SCORE_ALGAE_BARGE,
    STAY_STILL;
  }

  private Goal desiredGoal = Goal.STOW;

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
    return startEnd(() -> setGoal(goal), () -> setGoal(Goal.STOW))
        .withName("Superstructure " + goal);
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
      case COLLECT_CORAL -> {
        elevator.setTargetHeight(ElevatorGoal.COLLECT_CORAL);
        coralManip.setSolenoidState(Value.kForward);
        rotateWindmillWhenAtPosition(Value.kForward, WindmillGoal.COLLECT_CORAL);
        break;
      }
      case SCORE_L1_CORAL -> {
        elevator.setTargetHeight(ElevatorGoal.L1_CORAL);
        windmill.setTargetPosition(WindmillGoal.L1_CORAL);
        SmartDashboard.putBoolean("Is At Goal", windmill.isAtGoal(WindmillGoal.L1_CORAL));
        actuateCoralWhenAtPosition(Value.kForward, WindmillGoal.L1_CORAL);

        break;
      }
      case SCORE_L2_CORAL -> {
        elevator.setTargetHeight(ElevatorGoal.L2_CORAL);
        windmill.setTargetPosition(WindmillGoal.L2_CORAL);
        actuateCoralWhenAtPosition(Value.kReverse, WindmillGoal.L2_CORAL);

        break;
      }
      case GRAB_L2_ALGAE -> {
        elevator.setTargetHeight(ElevatorGoal.L2_ALGAE);
        windmill.setTargetPosition(WindmillGoal.L2_ALGAE);

        break;
      }
      case SCORE_L3_CORAL -> {
        elevator.setTargetHeight(ElevatorGoal.L3_CORAL);
        windmill.setTargetPosition(WindmillGoal.L3_CORAL);
        actuateCoralWhenAtPosition(Value.kReverse, WindmillGoal.L3_CORAL);

        break;
      }
      case GRAB_L3_ALGAE -> {
        elevator.setTargetHeight(ElevatorGoal.L3_ALGAE);
        windmill.setTargetPosition(WindmillGoal.L3_ALGAE);

        break;
      }
      case SCORE_ALGAE_BARGE -> {
        elevator.setTargetHeight(ElevatorGoal.ALGAE_BARGE);
        windmill.setTargetPosition(WindmillGoal.ALGAE_BARGE);

        break;
      }
      case ALGAE_PROCESSOR_AND_PRESTAGE -> {
        elevator.setTargetHeight(ElevatorGoal.ALGAE_PROCESSOR_AND_PRESTAGE);
        windmill.setTargetPosition(WindmillGoal.ALGAE_PROCESSOR_AND_PRESTAGE);

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
  }
}
