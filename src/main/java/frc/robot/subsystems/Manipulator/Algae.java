package frc.robot.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
  private final AlgaeIO io;
  public final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

  public Algae(AlgaeIO io) {
    this.io = io;

    switch (Constants.currentMode) {
        // allows to edit formats/vars for each mode
      case REAL:
      case REPLAY:
        // reevaluate vars and update -> hypotheticals
        break;
      case SIM:
        break;
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Algae", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void setVoltageLauncher(double volts) {
    io.setVoltageLauncher(volts);
  }

  public void stopLauncher() {
    io.stopOuter();
  }

  public void setVoltageHolding(double volts) {
    io.setVoltageHolding(volts);
  }

  public void stopHolding() {
    io.stopInner();
  }
}
