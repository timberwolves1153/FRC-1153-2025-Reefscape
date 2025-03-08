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
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Windmill extends SubsystemBase implements AutoCloseable {

  private WindmillIO windmillIo;
  public WindmillInputsAutoLogged windmillInputs;

  private ProfiledPIDController windmillPID;
  private TrapezoidProfile.Constraints windmillConstraints;
  private ArmFeedforward windmillFF;

  private Mechanism2d windmillMech2d;
  private MechanismRoot2d root;
  private MechanismLigament2d windmillLigament;

  public enum ProtoWindmillGoal {
    STOW(-11.07),
    COLLECT_CORAL(80.77),
    // PRESTAGE_ALGAE(131.57),
    L1_CORAL(-97.03),
    L2_CORAL(-141.67),
    L2_ALGAE(67.76),
    L3_CORAL(-141.67),
    L3_ALGAE(67.76),
    L4_CORAL(-141.67),
    ALGAE_PROCESSOR_AND_PRESTAGE(131.57),
    ALGAE_BARGE(-10.366);

    private double angleInDegrees;

    private ProtoWindmillGoal(double angleInDegrees) {
      this.angleInDegrees = angleInDegrees;
    }

    public double getPositionInDegrees() {
      return this.angleInDegrees;
    }
  }

  public enum WindmillGoal {
    // bounds are +100 and
    STOW(-135.07),
    COLLECT_CORAL(-41.5),
    // PRESTAGE_ALGAE(131.57),
    L1_CORAL(-240.29),
    L2_CORAL(-228.69),
    L2_ALGAE(-99.66),
    L3_CORAL(-228.69),
    L3_ALGAE(-99.66),
    L4_CORAL(-202.8),
    ALGAE_PROCESSOR_AND_PRESTAGE(-252.5),
    ALGAE_BARGE(-185);

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
    setTargetPositionDegrees(getDegreeGoalForCurrentRobot(degreeGoal));
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
    double robotSpecificGoal = getDegreeGoalForCurrentRobot(goal);
    double error = Math.abs(currentDegrees - robotSpecificGoal);
    return error < 5;
  }

  public double getDegreeGoalForCurrentRobot(WindmillGoal degreeGoal) {
    if (Constants.Bot.COMP.equals(Constants.currentBot)) {
      return degreeGoal.getPositionInDegrees();
    } else {
      if (degreeGoal.equals(WindmillGoal.STOW)) {
        return ProtoWindmillGoal.STOW.getPositionInDegrees();
      } else if (degreeGoal.equals(WindmillGoal.COLLECT_CORAL)) {
        return ProtoWindmillGoal.COLLECT_CORAL.getPositionInDegrees();
      } else if (degreeGoal.equals(WindmillGoal.L1_CORAL)) {
        return ProtoWindmillGoal.L1_CORAL.getPositionInDegrees();
      } else if (degreeGoal.equals(WindmillGoal.L2_CORAL)) {
        return ProtoWindmillGoal.L2_CORAL.getPositionInDegrees();
      } else if (degreeGoal.equals(WindmillGoal.L3_CORAL)) {
        return ProtoWindmillGoal.L3_CORAL.getPositionInDegrees();
      } else if (degreeGoal.equals(WindmillGoal.L4_CORAL)) { // I literally hate all of you so much
        return ProtoWindmillGoal.L3_CORAL.getPositionInDegrees();
      } else if (degreeGoal.equals(WindmillGoal.L2_ALGAE)) {
        return ProtoWindmillGoal.L2_ALGAE.getPositionInDegrees();
      } else if (degreeGoal.equals(WindmillGoal.L3_ALGAE)) {
        return ProtoWindmillGoal.L3_ALGAE.getPositionInDegrees();
      } else if (degreeGoal.equals(WindmillGoal.ALGAE_PROCESSOR_AND_PRESTAGE)) {
        return ProtoWindmillGoal.ALGAE_PROCESSOR_AND_PRESTAGE.getPositionInDegrees();
      } else if (degreeGoal.equals(WindmillGoal.ALGAE_BARGE)) {
        return ProtoWindmillGoal.ALGAE_BARGE.getPositionInDegrees();
      } else {
        return ProtoWindmillGoal.STOW.getPositionInDegrees();
      }
    }
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
