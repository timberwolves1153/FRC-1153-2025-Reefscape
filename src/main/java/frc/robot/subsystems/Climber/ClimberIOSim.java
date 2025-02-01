package frc.robot.subsystems.Climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO {

  private DCMotorSim climberSim;
  private double volts;

  public ClimberIOSim() {

    climberSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.03, 125.0), DCMotor.getNEO(1));

    volts = 0.0;
  }

  @Override
  public void updateInputs(ClimberInputs climberInputs) {
    climberInputs.climberCurrentAmps = climberSim.getCurrentDrawAmps();
    climberInputs.climberTempCelsius = 20;
    climberInputs.appliedVoltage = volts;
  }

  @Override
  public void setVoltage(final double voltage) {
    volts = voltage;
    climberSim.setInputVoltage(MathUtil.clamp(voltage, -4, 4));
  }

  @Override
  public void stop() {
    climberSim.setInputVoltage(0);
  }
}
