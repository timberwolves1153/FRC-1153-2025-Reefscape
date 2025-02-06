package frc.robot.subsystems.windmill;
/*NOTE FOR WHOEVER IS READING THIS:
 *  - Comments are gonna be there for my help
 *  - They'll rarely be deleted, unless abundant
 *  - Have Fun!
 */
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Windmill extends SubsystemBase implements AutoCloseable {

  private WindmillIO windmillIo;
  private WindmillInputsAutoLogged windmillInputs;

  private ProfiledPIDController windmillPID;
  private TrapezoidProfile.Constraints windmillConstraints;
  private ArmFeedforward windmillFF;

  private Mechanism2d windmillMech2d;
  private MechanismRoot2d root;
  private MechanismLigament2d windmillLigament;
  private SysIdRoutine sysIdRoutine;

  public enum WindmillGoal {
    STOW(0),
    COLLECT_CORAL(5),
    L1_CORAL(10),
    L2_CORAL(15),
    L2_ALGAE(20),
    L3_CORAL(25),
    L3_ALGAE(30),
    ALGAE_PROCESSOR(35),
    ALGAE_BARGE(40);

    private double angleInDegrees;

    private WindmillGoal(double angleInDegrees) {
      this.angleInDegrees = angleInDegrees;
    }

    public double getPositionInDegrees() {
      return this.angleInDegrees;
    }
  }

  public Windmill(WindmillIO io) {
    this.windmillIo = io;
    windmillInputs = new WindmillInputsAutoLogged();

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null, // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.setVoltage(voltage.in(Volts)),
                null, // No log consumer, since data is recorded by AdvantageKit
                this));

    windmillConstraints =
        new Constraints(
            2, 2); // check up with this, might be needed to change to different constants later

    windmillPID =
        new ProfiledPIDController(5, 0, 0, windmillConstraints); // set to what we need later
    windmillFF = new ArmFeedforward(2, 2, 0); // Check ^^ (Also what in the world is Ks Kg and Kv)

    windmillMech2d = new Mechanism2d(60, 60); // Find the right size for this
    root = windmillMech2d.getRoot("WindmillRoot", 30, 30); // Create root at the center
    windmillLigament =
        root.append(
            new MechanismLigament2d(
                "Windmill", 10, 0)); // Create ligament representing the windmill

    windmillInputs = new WindmillInputsAutoLogged();

    SmartDashboard.putData(
        "Windmill Mechanism", windmillMech2d); // Add Mechanism2d to SmartDashboard
  }

  public void setVoltage(double voltage) {
    windmillIo.setVoltage(voltage);
  }

  public void stop() {
    windmillIo.stop();
  }

  public void setTargetPosition(WindmillGoal degreeGoal) {
    setTargetPositionDegrees(degreeGoal.getPositionInDegrees());
  }

  public void setTargetPositionDegrees(double degrees) {
    double rotations = Units.degreesToRotations(degrees);
    windmillIo.setTargetPosition(rotations);
  }

  public void holdPosition() {
    windmillPID.setGoal(windmillInputs.absolutePositionRadians);

    windmillIo.setVoltage(
        windmillPID.calculate(
                windmillInputs.absolutePositionRadians, windmillInputs.absolutePositionRadians)
            + windmillFF.calculate(
                windmillInputs.absolutePositionRadians, windmillPID.getSetpoint().velocity));
  }

  public Command runcharaterizationForwardQ() {

    return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command runcharaterizationReverseQ() {

    return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command runcharaterizationForwardD() {

    return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command runcharaterizationReverseD() {

    return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the Mechanism2d with the current state of the windmill
    SmartDashboard.putData("Windmill Mechanism", windmillMech2d);

    windmillIo.updateInputs(windmillInputs);
    Logger.recordOutput("windmill position degrees", windmillInputs.absolutePositionDegrees);
    Logger.recordOutput("windmill position", windmillInputs.absolutePosition);
    Logger.processInputs("Windmill", windmillInputs);
    SmartDashboard.putNumber("windmill position degrees", windmillInputs.absolutePositionDegrees);
    SmartDashboard.putNumber(
        "windmill position rotations",
        Units.degreesToRotations(windmillInputs.absolutePositionDegrees));
    SmartDashboard.putNumber("windmill position radians", windmillInputs.absolutePositionRadians);
    windmillLigament.setAngle(windmillInputs.absolutePositionDegrees);
  }

  @Override
  public void close() throws Exception {
    windmillIo.close();
  }

  public Object runVolts(Voltage voltage) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runVolts'");
  }
}
