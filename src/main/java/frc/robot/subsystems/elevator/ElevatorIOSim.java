package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

  private ElevatorSim elevatorSim;
  private double volts;

  public ElevatorIOSim() {
    elevatorSim =
        new ElevatorSim(
            DCMotor.getFalcon500(2),
            12,
            Units.lbsToKilograms(17.966),
            Units.inchesToMeters(1.7567),
            Units.inchesToMeters(0),
            Units.inchesToMeters(63.0),
            true,
            Units.inchesToMeters(0));

    volts = 0.0;
  }

  @Override
  public void updateInputs(ElevatorInputs elevatorInputs) {
    elevatorSim.update(0.02);

    elevatorInputs.heightMeters = elevatorSim.getPositionMeters();
    elevatorInputs.elevatorCurrentAmps = elevatorSim.getCurrentDrawAmps();
    elevatorInputs.tempCelsius = 20;
    elevatorInputs.getAppliedVolts = volts;
  }

  @Override
  public void setVoltage(final double voltage) {
    volts = voltage;
    elevatorSim.setInputVoltage(MathUtil.clamp(voltage, -4, 4));

    // elevatorSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    elevatorSim.setInputVoltage(0);
    ;
  }

  public double getElevatorHeight() {
    return Units.metersToInches(elevatorSim.getPositionMeters());
  }
}
