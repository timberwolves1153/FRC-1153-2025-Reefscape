package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorInputs {
        public double positionInches = 0.0;
        public double percentOut = 0.0;
        public double pidSetpoint = 0.0;
        public double[] currentAmps = new double[] {};
    }

    public default void updateInputs(ElevatorInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void stop() {}

    public default void zeroElevatorEncoder() {}

    public default void setTargetHeight(double inches) {}
    
    
}
