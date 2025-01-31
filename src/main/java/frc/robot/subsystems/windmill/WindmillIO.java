package frc.robot.subsystems.windmill;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.AutoLog;

public interface WindmillIO {
  @AutoLog
  public class WindmillInputs {
    public double positionDegrees;
    public Rotation2d absolutePosition = new Rotation2d();
    public double absolutePositionRadians;
    public double absolutePositionDegrees;
    public double appliedVolts;
    public double current;
    public double velocityRadPerSec;

    // check if we need To either keep these empty or add values to it

  }

  public default void updateInputs(WindmillInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void holdPosition() {}

  public default void setTargetPosition(double degrees) {}

  public default void close() {}
}
