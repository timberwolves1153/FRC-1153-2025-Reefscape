package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorInputs {
    public double positionRots = 0.0;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] currentAmps = new double[] {};
    public double[] tempCelsius = new double[] {};
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
