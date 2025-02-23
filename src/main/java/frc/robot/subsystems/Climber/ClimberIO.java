package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberInputs {
    public double appliedVoltage = 0.0;
    public double climberCurrentAmps = 0.0;
    public double climberTempCelsius = 0.0;
  }

  public default void updateInputs(ClimberInputs climberInputs) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}
}
