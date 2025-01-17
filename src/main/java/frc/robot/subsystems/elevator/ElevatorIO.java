package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorInputs {
    public double elevatorCurrentAmps = 0.0;
    public double pidSetpoint = 0.0;
    public double heightInches = 0.0;
    public double getVoltageOut = 0.0;
    public double elevatorEncoderValue = 0.0;
  }

  public default void updateInputs(ElevatorInputs elevatorInputs) {}

  public default void setVoltage(double volts) {}

  public default void resetElevatorEncoder() {}

  public default void setTargetHeight() {}

  public default void stop() {}
}
