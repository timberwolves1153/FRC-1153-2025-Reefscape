package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorInputs {
    public double elevatorCurrentAmps = 0.0;
    public double heightMeters = 0.0;
    public double getAppliedVolts = 0.0;
    public double tempCelsius = 0.0;
    public boolean isSwitchTriggered = false;
  }

  public default void updateInputs(ElevatorInputs elevatorInputs) {}

  public default void setVoltage(double volts) {}

  public default void resetElevatorEncoder() {}

  public default void setTargetHeight(double inches) {}

  public default void stop() {}

  public default boolean isSwitchTriggered() {
    return false;
  }
}
