package frc.robot.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
  private final AlgaeIO io;
  private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

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
  public void runVoltsOuter(double volts) {
    io.setVoltageOuter(volts);
  }

  public void stopOuter() {
    io.stopOuter();
  }

  public void runVoltsInner(double volts) {
    io.setVoltageInner(volts);
  }

  public void stopInner() {
    io.stopInner();
  }
}
