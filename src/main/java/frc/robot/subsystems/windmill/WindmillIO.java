package frc.robot.subsystems.windmill;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface WindmillIO {
  @AutoLog
  public class WindmillInputs {
    public double rotations = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(WindmillInputs inputs) {}

  public default void setVoltage(Voltage volts) {}

  public default void stop() {}

  public default void holdPosition() {}

  public default void setTargetPosition(double rotations) {}

  public default void close() {}
}
