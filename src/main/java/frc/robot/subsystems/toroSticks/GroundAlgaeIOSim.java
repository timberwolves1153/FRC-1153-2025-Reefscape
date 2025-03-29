package frc.robot.subsystems.toroSticks;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class GroundAlgaeIOSim implements GroundAlgaeIO {

  private SingleJointedArmSim armSim;
  private Voltage volts;

  public GroundAlgaeIOSim() {

    volts = Voltage.ofBaseUnits(0.0, Volts);
    armSim =
        new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            0,
            84368.260313,
            Units.inchesToMeters(22),
            Math.PI / 2,
            Math.PI,
            false,
            Math.PI / 2);
  }

  @Override
  public void updateInputs(GroundAlgaeInputs inputs) {
    inputs.pivotAppliedVolts = volts.baseUnitMagnitude();
    inputs.pivotCurrentAmps = armSim.getCurrentDrawAmps();
  }

  @Override
  public void pivotUp() {

  }

  @Override
  public void pivotDown() {
    
  }
}
