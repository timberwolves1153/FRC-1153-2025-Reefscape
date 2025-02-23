package frc.robot.subsystems.Manipulator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
// import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class AlgaeIOSim implements AlgaeIO {
  private final DCMotorSim simOuter;
  private final DCMotorSim simInner;

  private double appliedVoltsOuter = 0.0;
  private double appliedVoltsInner = 0.0;

  public AlgaeIOSim() {
    simOuter =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 1, 1), DCMotor.getNEO(1));

    simInner =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 1, 1), DCMotor.getNEO(1));
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      setVoltageHolding(0);
      setVoltageLauncher(0);
    }

    inputs.outerAppliedVolts = simOuter.getInputVoltage();
    inputs.outerCurrentAmps = simOuter.getCurrentDrawAmps();

    inputs.innerAppliedVolts = simInner.getInputVoltage();
    inputs.innerCurrentAmps = simInner.getCurrentDrawAmps();
  }

  @Override
  public void setVoltageLauncher(double volts) {
    appliedVoltsOuter = MathUtil.clamp(volts, -12, 12); // can be edited
    simOuter.setInputVoltage(appliedVoltsOuter);
  }

  @Override
  public void stopOuter() {
    setVoltageLauncher(0);
  }

  @Override
  public void setVoltageHolding(double volts) {
    appliedVoltsInner = MathUtil.clamp(volts, -12, 12); // can be edited
    simInner.setInputVoltage(appliedVoltsInner);
  }

  @Override
  public void stopInner() {
    setVoltageHolding(0);
  }
}
