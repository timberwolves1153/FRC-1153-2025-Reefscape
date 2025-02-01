package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
  public final SysIdRoutine sysIdRoutine;

  public Elevator(ElevatorIO elevatorIO) {
    elevatorInputs = new ElevatorInputsAutoLogged();
    this.elevatorIO = elevatorIO;

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> elevatorIO.setVoltage(volts), null, this));

    switch (Constants.currentMode) {

        // Check values below to make sure they are accurate; might need to be changed later
      case REAL:
      case REPLAY:
        constraints = new TrapezoidProfile.Constraints(5, 10);
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
    elevatorIO.setVoltage(Voltage.ofBaseUnits(volts, Volts));
  }

  public void stop() {
    elevatorIO.stop();
  }

  public void updateMech2d() {
    elevatorLig2d.setLength(elevatorInputs.heightInches);
  }

  public void setTargetHeight(double inches) {
    elevatorIO.setTargetHeight(inches);
  }

  public void holdTargetHeight() {
    double calculatedVolts =
        profiledPIDController.calculate(elevatorInputs.heightInches, elevatorInputs.heightInches)
            + elevatorFF.calculate(profiledPIDController.getSetpoint().velocity);
    elevatorIO.setVoltage(Voltage.ofBaseUnits(calculatedVolts, Volts));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  // public double getElevatorHeight() {
  //   return elevatorIO.ge
  // }
  @Override
  public void periodic() {
    elevatorIO.updateInputs(elevatorInputs);
    elevatorLig2d.setLength((elevatorInputs.heightInches));
    Logger.processInputs("Elevator", elevatorInputs);
    Logger.recordOutput("Elevator/Mechanism2D", elevatorMech2d);
    Logger.recordOutput("Elevator Height", elevatorInputs.leaderRotations);
  }
}
