package frc.robot.subsystems.windmill;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WindmillIOSim implements WindmillIO {

  private SingleJointedArmSim windmillSim;
  private PIDController windmillPID;
  private CANcoder encoder;

  public WindmillIOSim() {

    windmillSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), // prototype change
            134.4, // Could be 100.8
            11, // remember to check what this is
            Units.inchesToMeters(27.5),
            Units.degreesToRadians(15),
            Units.degreesToRadians(270),
            true,
            Units.degreesToRadians(0));

    // GET ALL OF THESE FROM HE CAD WHEN IT IS DONE

    windmillPID = new PIDController(0.05, 0, 0);

    encoder = new CANcoder(44);
  }

  @Override
  public void updateInputs(WindmillInputs inputs) {
    windmillSim.update(.02);

    getAbsolutePosition();

    // double check if this is all that you need to do in the updateInputs method
  }

  @Override
  public void setVoltage(double volts) {
    windmillSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    windmillSim.setInputVoltage(0);
  }

  public double getWindmillPosition() {
    return Units.radiansToDegrees(windmillSim.getAngleRads());
  }

  public double getAbsolutePosition() {
    return encoder.getAbsolutePosition().getValueAsDouble();
  }

  public void close() {}
}
