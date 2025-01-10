package frc.robot.subsystems.Windmill;
/*NOTE FOR WHOEVER IS READING THIS:
 *  - Comments are gonna be there for my help
 *  - They'll rarely be deleted, unless abundant
 *  - Have Fun!
 */
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Windmill.WindmillIO.WindmillInputs;

public class Windmill extends SubsystemBase {


    private WindmillIO windmillIo;
    private WindmillInputs windmillInputs;

    private ProfiledPIDController windmillPID;
    private TrapezoidProfile.Constraints windmillConstraints;
    private ArmFeedforward windmillFF;

    private Mechanism2d meched;


    public Windmill(WindmillIO io) {
        this.windmillIo = io;
        windmillConstraints = new Constraints(2, 2); //check up with this, might be needed to change to different constants later

        windmillPID = new ProfiledPIDController(0, 0, 0, windmillConstraints); //set to what we need later
        windmillFF = new ArmFeedforward(0, 0, 0); // Check ^^ (Also what in the world is Ks Kg and Kv)

    

    }

    public void setVoltage(double voltage) {
        windmillIo.setVoltage(voltage);
    }

    public void stop() {
        windmillIo.stop();
    }

    public void setTargetPosition(double degrees) {
        windmillPID.setGoal(Units.degreesToRadians(degrees)); // mkae sure we know whats happening here, with the degrees at least
        
        windmillIo.setVoltage(
            windmillPID.calculate(
                windmillInputs.absolutePositionRadians,
                Units.degreesToRadians(degrees))
            +   windmillFF.calculate(
                Units.degreesToRadians(degrees),
                windmillPID.getSetpoint().velocity));
    }


    public void holdPosition() {
        windmillPID.setGoal(windmillInputs.absolutePositionRadians);

        windmillIo.setVoltage(
            windmillPID.calculate(
                windmillInputs.absolutePositionRadians,
                windmillInputs.absolutePositionRadians)   
        +   windmillFF.calculate(
            windmillInputs.absolutePositionRadians,
             windmillPID.getSetpoint().velocity));
    }

    


}
