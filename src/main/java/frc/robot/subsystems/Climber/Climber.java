package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO climberIO;
  private final ClimberInputsAutoLogged climberInputs = new ClimberInputsAutoLogged();

  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;
  }

  public void setVoltage(double volts) {
    climberIO.setVoltage(volts);
  }

  public void stop(double volts) {
    climberIO.setVoltage(0);
    ;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);
    Logger.processInputs("Climber", climberInputs);
  }
}
