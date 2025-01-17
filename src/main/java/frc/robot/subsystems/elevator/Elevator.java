package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  public ElevatorIO elevatorIO;
  public ElevatorInputsAutoLogged elevatorInputs;
  public PIDController elevatorPID;
  public TrapezoidProfile.Constraints constraints;
  public ProfiledPIDController profiledPIDController;
  public ElevatorFeedforward elevatorFF;

  private Mechanism2d elevatorMech2d;
  private MechanismRoot2d elevatorRoot2d;
  private MechanismLigament2d elevatorLig2d;

  public Elevator(ElevatorIO elevatorIO) {

    switch (Constants.currentMode) {

        // Check values below to make sure they are accurate; might need to be changed later
      case REAL:
      case REPLAY:
        constraints = new TrapezoidProfile.Constraints(0, 0);
        profiledPIDController = new ProfiledPIDController(0, 0, 0, constraints);
        elevatorFF = new ElevatorFeedforward(0, 0, 0);
        elevatorPID = new PIDController(0, 0, 0);
        break;

      case SIM:
        constraints = new TrapezoidProfile.Constraints(0, 0);
        profiledPIDController = new ProfiledPIDController(0, 0, 0, constraints);
        elevatorFF = new ElevatorFeedforward(0, 0, 0);
        elevatorPID = new PIDController(0, 0, 0);
        break;

      default:
        constraints = new TrapezoidProfile.Constraints(0, 0);
        profiledPIDController = new ProfiledPIDController(0, 0, 0, constraints);
        elevatorFF = new ElevatorFeedforward(0, 0, 0);
        elevatorPID = new PIDController(0, 0, 0);
        break;
    }

    elevatorMech2d = new Mechanism2d(10.0, 39.0);
    elevatorRoot2d = elevatorMech2d.getRoot("Elevator Root", 0, 0);
    elevatorLig2d = elevatorRoot2d.append(new MechanismLigament2d("Elevator", 63.0, 90));

    elevatorInputs = new ElevatorInputsAutoLogged();
    this.elevatorIO = elevatorIO;
  }

  public void setVoltage(double volts) {
    elevatorIO.setVoltage(volts);
  }

  public void stop() {
    elevatorIO.setVoltage(0);
  }

  public void updateMech2d() {
    elevatorLig2d.setLength(Units.inchesToMeters(elevatorInputs.heightInches));
  }

  //
  // public double inchesToEncoderTicks(double setpoint) {
  //     double encoderTicks = 42.0;
  //     double shaftRadius = Units.metersToInches(0.008);

  //     double distancePerRevolution = shaftRadius * 2 * Math.PI;
  //     double conversion = distancePerRevolution/encoderTicks;

  //     return setpoint/conversion;
  // }

  // public void setTargetHeight(double inches) {
  //     profiledPIDController.setGoal(inchesToEncoderTicks(inches));

  //     elevatorIO.setVoltage(
  //         profiledPIDController.calculate(inchesToEncoderTicks(elevatorInputs.heightInches)) +
  //         elevatorFF.calculate(profiledPIDController.getSetpoint().velocity));

  // }

  // public void holdTargetHeight() {
  //     profiledPIDController.setGoal(inchesToEncoderTicks(elevatorInputs.heightInches));

  //     elevatorIO.setVoltage(
  //         profiledPIDController.calculate(inchesToEncoderTicks(elevatorInputs.heightInches)) +
  //         elevatorFF.calculate(profiledPIDController.getSetpoint().velocity));
  // }

  @Override
  public void periodic() {
    elevatorIO.updateInputs(elevatorInputs);
    Logger.processInputs("Elevator Inputs", elevatorInputs);
    //  Logger.processInputs("Elevator Setpoint", profiledPIDController.getSetpoint());
  }
}
