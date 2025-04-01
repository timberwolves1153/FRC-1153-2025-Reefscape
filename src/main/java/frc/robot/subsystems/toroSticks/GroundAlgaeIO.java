package frc.robot.subsystems.toroSticks;

import org.littletonrobotics.junction.AutoLog;

public interface GroundAlgaeIO {

  @AutoLog
  public class GroundAlgaeInputs {
    public double pivotAppliedVolts = 0.0; // input
    public double pivotCurrentAmps = 0.0;

    public double rollerAppliedVolts = 0.0; // input
    public double rollerCurrentAmps = 0.0;

    public double pivotRotations = 0.0;
  }

  public default void updateInputs(GroundAlgaeInputs inputs) {}

  public default void pivotDown() {}

  public default void pivotUp() {}

  public default void deploy() {}

  public default void stow() {}

  public default void pivotStop() {}

  public default void intake() {}

  public default void stopRollers() {}

  public default void outtake() {}

  public default void setPivotPostion(double rotations) {}

  public default void resetPivotEncoder() {}
}
