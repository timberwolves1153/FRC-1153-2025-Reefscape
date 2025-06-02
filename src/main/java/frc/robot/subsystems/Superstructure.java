package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GamePiece;
import frc.robot.FieldConstants;
import frc.robot.Interpolation.InterpolatingDouble;
import frc.robot.Interpolation.WindmillTable;
import frc.robot.subsystems.Manipulator.Algae;
import frc.robot.subsystems.Manipulator.Coral;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.windmill.Windmill;
import frc.robot.subsystems.windmill.Windmill.WindmillGoal;

public class Superstructure extends SubsystemBase {

  public enum Goal {
    STOW,
    COLLECT,
    PRESTAGE_ALGAE,
    L1,
    L2,
    L3,
    L4,
    GROUND,
    STAY_STILL,
    BACKSIDE_L1,
    LYNK_L1_READY,
    LYNK_L1_SCORE,
    CLIMB;
  }

  private Goal desiredGoal = Goal.STOW;
  private GamePiece currentGamePiece = GamePiece.CORAL;

  private Elevator elevator;
  private Windmill windmill;
  private Drive drive;
  private Coral coralManip;
  private Algae algaeManip;
  private WindmillTable windmillTable;
  // private WindmillTable launcherMap;
  private Timer goalTimer = new Timer();

  public Superstructure(
      Elevator elevator, Windmill windmill, Coral coralManip, Algae algaeManip, Drive drive) {
    this.elevator = elevator;
    this.windmill = windmill;
    this.coralManip = coralManip;
    this.algaeManip = algaeManip;
    this.drive = drive;
    // launcherMap = new WindmillTable();
    windmillTable = new WindmillTable();
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

  public void setAutoGoalCommand(Goal goal) {
    setGoal(goal);
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

  public void setAutoGamepieceCommand(GamePiece gamePiece) {
    setGamepiece(gamePiece);
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

  public void interpolateAlgaeShot() {
    algaeManip.setVoltageLauncher(
        windmillTable.launcherMap.getInterpolated(
                new InterpolatingDouble(FieldConstants.getNearestCage(drive.getPose())))
            .value);
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
        boolean isRedAlliance =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        // okay so now zena goes absolutley crazy at the coral station and slams that coral in in
        // 0.000000000001 seconds
        // and then we win world champs and theyre all like OOOOOOOHHHHH THAT GOES CRAZY WHO LET
        // 1153 COOK THAT HARD WHOOOOOOOOO

        // if robot-stick side is closer to station
        // go to x setoint

        // if robot-scoring side is closer to station
        // go to y settoint
        if (FieldConstants.getNearestCoralStation(drive.getPose())
                .equals(FieldConstants.CoralStation.rightCenterFace)
            && isRedAlliance) {
          if (drive.getPose().getRotation().getDegrees() < 142
              && drive.getPose().getRotation().getDegrees() > -38) {
            elevator.setTargetHeight(ElevatorGoal.COLLECT_CORAL_FRONT);
            windmill.setTargetPosition(WindmillGoal.COLLECT_CORAL_FRONT);
          } else {
            elevator.setTargetHeight(ElevatorGoal.COLLECT_CORAL);
            windmill.setTargetPosition(WindmillGoal.COLLECT_CORAL);
          }

        } else if (FieldConstants.getNearestCoralStation(drive.getPose())
                .equals(FieldConstants.CoralStation.leftCenterFace)
            && isRedAlliance) {
          if (drive.getPose().getRotation().getDegrees() < 174
              && drive.getPose().getRotation().getDegrees() > -5) {
            elevator.setTargetHeight(ElevatorGoal.COLLECT_CORAL);
            windmill.setTargetPosition(WindmillGoal.COLLECT_CORAL);
          } else {
            elevator.setTargetHeight(ElevatorGoal.COLLECT_CORAL_FRONT);
            windmill.setTargetPosition(WindmillGoal.COLLECT_CORAL_FRONT);
          }
        } else if (FieldConstants.getNearestCoralStation(drive.getPose())
            .equals(FieldConstants.CoralStation.leftCenterFace)) {
          if (drive.getPose().getRotation().getDegrees() < 174
              && drive.getPose().getRotation().getDegrees() > -5) {
            elevator.setTargetHeight(ElevatorGoal.COLLECT_CORAL_FRONT);
            windmill.setTargetPosition(WindmillGoal.COLLECT_CORAL_FRONT);
          } else {
            elevator.setTargetHeight(ElevatorGoal.COLLECT_CORAL);
            windmill.setTargetPosition(WindmillGoal.COLLECT_CORAL);
          }
        } else if (FieldConstants.getNearestCoralStation(drive.getPose())
            .equals(FieldConstants.CoralStation.rightCenterFace)) {
          if (drive.getPose().getRotation().getDegrees() < 142
              && drive.getPose().getRotation().getDegrees() > -38) {
            elevator.setTargetHeight(ElevatorGoal.COLLECT_CORAL);
            windmill.setTargetPosition(WindmillGoal.COLLECT_CORAL);
          } else {
            elevator.setTargetHeight(ElevatorGoal.COLLECT_CORAL_FRONT);
            windmill.setTargetPosition(WindmillGoal.COLLECT_CORAL_FRONT);
          }
        }
        break;
      }
      case L1 -> {
        if (GamePiece.CORAL.equals(getGamePiece())) {
          elevator.setTargetHeight(ElevatorGoal.L1_CORAL);
          windmill.setTargetPosition(WindmillGoal.L1_CORAL);
          SmartDashboard.putBoolean("Is At Goal", windmill.isAtGoal(WindmillGoal.L1_CORAL));
          actuateCoralWhenAtPosition(Value.kForward, WindmillGoal.L1_CORAL);
        } else if ((GamePiece.ALGAE.equals(getGamePiece()))) {
          elevator.setTargetHeight(ElevatorGoal.ALGAE_PRESTAGE);
          // coralManip.setSolenoidState(Value.kForward);
          // windmill.setTargetPosition(WindmillGoal.ALGAE_PROCESSOR_AND_PRESTAGE);
          windmill.setTargetPosition(WindmillGoal.ALGAE_PRESTAGE);
        } else { // default is coral
          elevator.setTargetHeight(ElevatorGoal.L1_CORAL);
          windmill.setTargetPosition(WindmillGoal.L1_CORAL);
          SmartDashboard.putBoolean("Is At Goal", windmill.isAtGoal(WindmillGoal.L1_CORAL));
          actuateCoralWhenAtPosition(Value.kForward, WindmillGoal.L1_CORAL);
        }
        break;
      }
      case BACKSIDE_L1 -> {
        elevator.setTargetHeight(ElevatorGoal.BACKSIDE_L1);
        windmill.setTargetPosition(WindmillGoal.BACKSIDE_L1);
      }
      case LYNK_L1_READY -> {
        elevator.setTargetHeight(ElevatorGoal.LYNK_L1_READY);
        windmill.setTargetPosition(WindmillGoal.LYNK_L1_READY);
      }
      case LYNK_L1_SCORE -> {
        elevator.setTargetHeight(ElevatorGoal.LYNK_L1_SCORE);
        windmill.setTargetPosition(WindmillGoal.LYNK_L1_SCORE);
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
      case L4 -> {
        if (GamePiece.CORAL.equals(getGamePiece())) {
          elevator.setTargetHeight(ElevatorGoal.L4_CORAL);
          windmill.setTargetPosition(WindmillGoal.L4_CORAL);
        } else if (GamePiece.ALGAE.equals(getGamePiece())) {
          elevator.setTargetHeight(ElevatorGoal.ALGAE_BARGE);
          // windmill.setTargetPositionDegrees(windmillTable.windmillMap.getInterpolated(new
          // InterpolatingDouble(FieldConstants.getNearestCage(drive.getPose()))).value);
          // windmill.setTargetPosition(WindmillGoal.ALGAE_BARGE);
          windmill.setTargetPositionDegrees(
              windmillTable.windmillMap.getInterpolated(
                      new InterpolatingDouble(FieldConstants.getNearestCage(drive.getPose())))
                  .value);
        } else {
          elevator.setTargetHeight(ElevatorGoal.L4_CORAL);
          windmill.setTargetPosition(WindmillGoal.L4_CORAL);
        }

        break;
      }
      case CLIMB -> {
        elevator.setTargetHeight(ElevatorGoal.CLIMB);
        windmill.setTargetPosition(WindmillGoal.CLIMB);
      }
      case GROUND -> {
        elevator.setTargetHeight(ElevatorGoal.ALGAE_GROUND);
        windmill.setTargetPosition(WindmillGoal.ALGAE_GROUND);
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
