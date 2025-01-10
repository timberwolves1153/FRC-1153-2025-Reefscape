package frc.robot.subsystems.Windmill;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;


public interface WindmillIO {
    @AutoLog
    public class WindmillInputs {
        public double positionDegrees = 0;
        public Rotation2d absolutePosition = new Rotation2d();
        public double absolutePositionRadians = 0;
        public double appliedVolts = 0;

    }

    public default void updateInputs(WindmillInputs inputs) {}

    public default void setVoltage(double volts) {}
    
    public default void stop() {}

    public default void holdPosition() {}

    public default void setTargetPosition(double degrees) {}


}
