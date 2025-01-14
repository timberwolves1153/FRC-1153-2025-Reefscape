package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void stop() {}

    public default void zeroElevatorEncoder() {}

    public default void setTargetHeight(double inches) {}
    
    
}
