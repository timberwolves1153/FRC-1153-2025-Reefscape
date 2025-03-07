package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

  private ElevatorSim elevatorSim;
  private Voltage volts;
  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(5.0, 10.0);

  ProfiledPIDController profiledPIDController = new ProfiledPIDController(40, 0, 0.1, constraints);

  ElevatorFeedforward elevatorFF =
      new ElevatorFeedforward(0, 0.06, (DCMotor.getFalcon500(1).KvRadPerSecPerVolt * 1.7567) / 12);

  public ElevatorIOSim() {
    elevatorSim =
        new ElevatorSim(
            DCMotor.getFalcon500(2),
            12,
            Units.lbsToKilograms(17.966),
            Units.inchesToMeters(1.751),
            0,
            Units.inchesToMeters(23),
            true,
            0);

    volts = Voltage.ofBaseUnits(0.0, Volts);
  }

  @Override
  public void updateInputs(ElevatorInputs elevatorInputs) {
    elevatorSim.update(0.02);

    elevatorInputs.heightInches = Units.metersToInches(elevatorSim.getPositionMeters());
    elevatorInputs.elevatorCurrentAmps = elevatorSim.getCurrentDrawAmps();
    elevatorInputs.tempCelsius = 20;
    elevatorInputs.getAppliedVolts = volts.baseUnitMagnitude();
    elevatorInputs.leaderRotations = 0;
  }

  @Override
  public void setVoltage(final Voltage voltage) {
    volts = voltage;
    double simVolts = voltage.baseUnitMagnitude();
    elevatorSim.setInputVoltage(MathUtil.clamp(simVolts, -12, 12));
  }

  @Override
  public void stop() {
    elevatorSim.setInputVoltage(0);
  }

  @Override
  public void setTargetHeight(double inches) {
    double calculatedVolts =
        profiledPIDController.calculate(
                elevatorSim.getPositionMeters(), Units.inchesToMeters(inches))
            + elevatorFF.calculate(profiledPIDController.getSetpoint().velocity);
    setVoltage(Voltage.ofBaseUnits(calculatedVolts, Volts));
  }

  public double getElevatorHeight() {
    return Units.metersToInches(elevatorSim.getPositionMeters());
  }
}
