package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {

  public ElevatorIO elevatorIO;
  public ElevatorInputsAutoLogged elevatorInputs;
  // public PIDController elevatorPID;
  public TrapezoidProfile.Constraints constraints;
  public ProfiledPIDController profiledPIDController;
  public ElevatorFeedforward elevatorFF;

  private LoggedMechanism2d elevatorMech2d;
  private LoggedMechanismRoot2d elevatorRoot2d;
  private LoggedMechanismLigament2d elevatorLig2d;

  private final Rotation2d elev_angle = Rotation2d.fromDegrees(90);

  public Elevator(ElevatorIO elevatorIO) {
    elevatorInputs = new ElevatorInputsAutoLogged();
    this.elevatorIO = elevatorIO;

    switch (Constants.currentMode) {

        // Check values below to make sure they are accurate; might need to be changed later
      case REAL:
      case REPLAY:
        constraints = new TrapezoidProfile.Constraints(0, 0);
        profiledPIDController = new ProfiledPIDController(0, 0, 0, constraints);
        elevatorFF = new ElevatorFeedforward(0, 0, 0);
        break;

      case SIM:
        constraints = new TrapezoidProfile.Constraints(5.0, 10.0);

        profiledPIDController = new ProfiledPIDController(40, 0, 0.1, constraints);

        elevatorFF =
            new ElevatorFeedforward(
                0, 0.06, (DCMotor.getFalcon500(1).KvRadPerSecPerVolt * 1.7567) / 12);

        break;

      default:
        profiledPIDController = new ProfiledPIDController(0, 0, 0, new Constraints(5, 10));
        elevatorFF = new ElevatorFeedforward(0, 0, 0);
        break;
    }

    elevatorMech2d = new LoggedMechanism2d(3, Units.feetToMeters(6));
    elevatorRoot2d =
        elevatorMech2d.getRoot(
            "Elevator", (3.0 / 2.0) + Units.inchesToMeters(9.053), Units.inchesToMeters(12.689));
    elevatorLig2d =
        new LoggedMechanismLigament2d(
            "Elevator Lig", Units.inchesToMeters(80), elev_angle.getDegrees());

    elevatorRoot2d.append(elevatorLig2d);
  }

  public void setVoltage(double volts) {
    elevatorIO.setVoltage(volts);
  }

  public void stop() {
    elevatorIO.setVoltage(0);
  }

  public void updateMech2d() {
    elevatorLig2d.setLength(elevatorInputs.heightMeters);
  }

  public void setTargetHeight(double inches) {
    profiledPIDController.setGoal(Units.inchesToMeters(inches));

    elevatorIO.setVoltage(
        profiledPIDController.calculate(elevatorInputs.heightMeters)
            + elevatorFF.calculate(profiledPIDController.getSetpoint().velocity));
  }

  public void holdTargetHeight() {
    profiledPIDController.setGoal(elevatorInputs.heightMeters);

    elevatorIO.setVoltage(
        profiledPIDController.calculate(elevatorInputs.heightMeters)
            + elevatorFF.calculate(profiledPIDController.getSetpoint().velocity));
  }

  @Override
  public void periodic() {
    elevatorIO.updateInputs(elevatorInputs);
    Logger.processInputs("Elevator", elevatorInputs);
    Logger.recordOutput("Elevator/Mechanism2D", elevatorMech2d);
    elevatorLig2d.setLength((elevatorInputs.heightMeters));
  }
}
