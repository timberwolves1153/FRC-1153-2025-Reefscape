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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public final double gearRatio = 7.1429;
  public final double pitchDiameter = 1.751;

  public enum ElevatorGoal {
    STOW(.25),
    L1_CORAL(0.25),
    L2_CORAL(0.25),
    L2_ALGAE(0.25),
    L3_CORAL(16.375),
    L3_ALGAE(16.5),
    COLLECT_CORAL(0.25),
    ALGAE_PROCESSOR_AND_PRESTAGE(0.25),
    // ALGAE_PROCESSOR(0.25),
    ALGAE_BARGE(23);

    private double heightInInches;

    private ElevatorGoal(double heightInInches) {
      this.heightInInches = heightInInches;
    }

    public double getHeightInInches() {
      return this.heightInInches;
    }
  }

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

  public void setTargetHeight(ElevatorGoal heightGoal) {
    setTargetHeightInches(heightGoal.getHeightInInches());
  }

  public void setTargetHeightInches(double inches) {
    double rots = inchesToRotations(inches);
    elevatorIO.setTargetHeight(rots);
  }

  public void setTargetHeightRotations(double rotations) {
    elevatorIO.setTargetHeight(rotations);
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

  public double inchesToRotations(double inches) {
    return (7.1429 * inches) / (Math.PI * pitchDiameter);
  }

  public double rotationsToInches(double rotations) {
    return (rotations / gearRatio) * (Math.PI * 2 * pitchDiameter);
  }

  @Override
  public void periodic() {
    elevatorIO.updateInputs(elevatorInputs);
    Logger.processInputs("Elevator", elevatorInputs);
    Logger.recordOutput("Elevator/Mechanism2D", elevatorMech2d);
    Logger.recordOutput("Elevator Height", elevatorInputs.leaderRotations);
    elevatorLig2d.setLength(Units.inchesToMeters(elevatorInputs.heightInches));
    SmartDashboard.putNumber("elevator height", elevatorInputs.heightInches);
    SmartDashboard.putNumber("elevator rotations", elevatorInputs.leaderRotations);
  }
}
