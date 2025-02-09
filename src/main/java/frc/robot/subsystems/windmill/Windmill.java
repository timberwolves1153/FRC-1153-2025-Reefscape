package frc.robot.subsystems.windmill;
/*NOTE FOR WHOEVER IS READING THIS:
 *  - Comments are gonna be there for my help
 *  - They'll rarely be deleted, unless abundant
 *  - Have Fun!
 */

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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

  public enum WindmillGoal {
    STOW(56),
    COLLECT_CORAL(185),
    L1_CORAL(-31.46),
    L2_CORAL(-77.5),
    L2_ALGAE(20),
    L3_CORAL(-77.5),
    L3_ALGAE(30),
    ALGAE_PROCESSOR(-80),
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
  }

  public void setVoltage(double voltage) {
    windmillIo.setVoltage(Voltage.ofBaseUnits(voltage, Volts));
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

  public Command setTargetPositionCommand(WindmillGoal degreeGoal) {
    return startEnd(() -> setTargetPosition(degreeGoal), () -> setTargetPosition(WindmillGoal.STOW))
        .withName("windmill " + degreeGoal);
  }

  public boolean isAtGoal(WindmillGoal goal) {
    double currentDegrees = Units.rotationsToDegrees(windmillInputs.rotations);
    double error = Math.abs(currentDegrees - goal.angleInDegrees);
    return error < 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update and process our inputs
    windmillIo.updateInputs(windmillInputs);
    Logger.processInputs("Windmill", windmillInputs);

    // conversions
    double degrees = Units.rotationsToDegrees(windmillInputs.rotations);
    double rads = Units.rotationsToRadians(windmillInputs.rotations);
    Rotation2d position = Rotation2d.fromRotations(windmillInputs.rotations);

    // Record our outputs
    Logger.recordOutput("windmill position degrees", degrees);
    Logger.recordOutput("windmill position", windmillInputs.rotations);

    // Add values to smart Dashboard
    SmartDashboard.putNumber("windmill position degrees", degrees);
    SmartDashboard.putNumber("windmill position rotations", windmillInputs.rotations);
    SmartDashboard.putNumber("windmill position radians", rads);

    // Update the simulation ligaments
    windmillLigament.setAngle(position);
  }

  @Override
  public void close() throws Exception {
    windmillIo.close();
  }
}
