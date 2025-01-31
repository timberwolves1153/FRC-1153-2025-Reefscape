package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Voltage;
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

  public default void setVoltage(Voltage volts) {}

  public default void resetElevatorEncoder() {}

  public default void setTargetHeight(double inches) {}

  public default void stop() {}
}
