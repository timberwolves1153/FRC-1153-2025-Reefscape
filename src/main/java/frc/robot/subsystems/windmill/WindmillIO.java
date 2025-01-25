package frc.robot.subsystems.windmill;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WindmillIO {
  @AutoLog
  public class WindmillInputs {
    public double positionDegrees = 0;
    public Rotation2d absolutePosition = new Rotation2d();
    public double absolutePositionRadians = 0;
    public double appliedVolts = 0;
    public double current = 0;

    // check if we need To either keep these empty or add values to it

  }

  public default void updateInputs(WindmillInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void holdPosition() {}

  public default void setTargetPosition(double degrees) {}

  public default void close() {}
}
