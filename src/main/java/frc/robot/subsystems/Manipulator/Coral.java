package frc.robot.subsystems.Manipulator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Coral extends SubsystemBase{
    private final CoralIO io;
    private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();

    public Coral(CoralIO io){
        this.io = io;

         switch (Constants.currentMode) {
            //allows to edit formats/vars for each mode
            case REAL:
            case REPLAY:
            //reevaluate vars and update -> hypotheticals
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
    }

    /** Run open loop at the specified voltage. */
    public void runVolts(double volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        io.stop();
      }

    public void setSolenoid(DoubleSolenoid.Value value){
        io.setSolenoid(value);
    }

}
