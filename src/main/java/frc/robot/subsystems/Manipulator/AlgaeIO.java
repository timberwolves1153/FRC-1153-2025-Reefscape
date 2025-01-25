package frc.robot.subsystems.Manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {

  @AutoLog
  public static class AlgaeIOInputs {
    public double appliedVolts = 0.0; // input
    public double currentAmps = 0.0;
  }

  public default void updateInputs(AlgaeIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Stop in open loop. */
  public default void stop() {}
}
