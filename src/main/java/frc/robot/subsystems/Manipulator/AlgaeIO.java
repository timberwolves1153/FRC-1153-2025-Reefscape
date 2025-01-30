package frc.robot.subsystems.Manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {

  @AutoLog
  public static class AlgaeIOInputs {
    public double outerAppliedVolts = 0.0; // input
    public double outerCurrentAmps = 0.0;

    public double innerAppliedVolts = 0.0; // input
    public double innerCurrentAmps = 0.0;
  }

  public default void updateInputs(AlgaeIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltageOuter(double volts) {}

  /** Stop in open loop. */
  public default void stopOuter() {}

  public default void setVoltageInner(double volts) {}

  /** Stop in open loop. */
  public default void stopInner() {}
}
