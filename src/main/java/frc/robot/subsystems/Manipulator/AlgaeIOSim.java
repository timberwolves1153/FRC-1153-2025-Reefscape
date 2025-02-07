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

  public AlgaeIOSim(DCMotor motor, double reduction, double momentOfInertia) {
    simOuter =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, momentOfInertia, reduction), motor);

    simInner =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, momentOfInertia, reduction), motor);
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      setVoltageOuter(0);
      setVoltageInner(0);
    }

    inputs.outerAppliedVolts = appliedVoltsOuter;
    inputs.outerCurrentAmps = simOuter.getCurrentDrawAmps();

    inputs.innerAppliedVolts = appliedVoltsInner;
    inputs.innerCurrentAmps = simInner.getCurrentDrawAmps();
  }

  @Override
  public void setVoltageOuter(double volts) {
    appliedVoltsOuter = MathUtil.clamp(volts, -12, 12); // can be edited
    simOuter.setInputVoltage(appliedVoltsOuter);
  }

  @Override
  public void stopOuter() {
    setVoltageOuter(0);
  }

  @Override
  public void setVoltageInner(double volts) {
    appliedVoltsInner = MathUtil.clamp(volts, -12, 12); // can be edited
    simInner.setInputVoltage(appliedVoltsInner);
  }

  @Override
  public void stopInner() {
    setVoltageInner(0);
  }
}
