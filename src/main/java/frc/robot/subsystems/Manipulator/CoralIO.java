package frc.robot.subsystems.Manipulator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public interface CoralIO {

    @AutoLog
    public static class CoralIOInputs{
        public double appliedVolts = 0.0; //input 
        public double currentAmps = 0.0;

    }

    public default void updateInputs(CoralIOInputs inputs) {}

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double volts) {}

    
    /** Stop in open loop. */
    public default void stop() {}

    public default void setSolenoid(DoubleSolenoid.Value value) {}
} 