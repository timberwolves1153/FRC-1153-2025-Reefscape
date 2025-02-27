package frc.robot.subsystems.Manipulator;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import org.littletonrobotics.junction.Logger;

public class Coral extends SubsystemBase {
  private final CoralIO io;
  private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();
  private GamePiece currentGamePiece = GamePiece.CORAL;

  public Coral(CoralIO io) {
    this.io = io;

    SmartDashboard.putNumber("output speed", -3);

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
    Logger.processInputs("Coral", inputs);
    SmartDashboard.putString("SolenoidPosition", getSolenoidState().toString());
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public Command jiggle() {
    return Commands.sequence(
        Commands.runOnce(() -> io.setVoltage(4)),
        new WaitCommand(0.2),
        Commands.runOnce(() -> io.setVoltage(-4)),
        new WaitCommand(0.2),
        Commands.runOnce(() -> io.setVoltage(4)),
        new WaitCommand(0.2),
        Commands.runOnce(() -> io.setVoltage(-4)),
        new WaitCommand(0.2),
        Commands.runOnce(() -> io.setVoltage(0.1)));
  }

  public void stop() {
    io.stop();
  }

  public void toggleSolenoid() {
    io.setSolenoid();
  }

  public void setSolenoidState(Value position) {
    io.setSolenoidState(position);
  }

  public boolean isAtGoal(Value position) {
    return position == io.getSolenoidState();
  }

  public Value getSolenoidState() {
    return io.getSolenoidState();
  }

  public GamePiece getCurrentGamePiece() {
    return this.currentGamePiece;
  }

  public void setCurrentGamePiece(GamePiece gamePiece) {
    this.currentGamePiece = gamePiece;
  }
}
