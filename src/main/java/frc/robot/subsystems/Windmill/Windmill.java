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
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Windmill.WindmillIO.WindmillInputs;

public class Windmill extends SubsystemBase {


    private WindmillIO windmillIo;
    private WindmillInputs windmillInputs;

    private ProfiledPIDController windmillPID;
    private TrapezoidProfile.Constraints windmillConstraints;
    private ArmFeedforward windmillFF;

    private Mechanism2d windmillMech2d;
    private MechanismRoot2d root;
    private MechanismLigament2d windmillLigament;



    public Windmill(WindmillIO io) {
        this.windmillIo = io;
        windmillConstraints = new Constraints(2, 2); //check up with this, might be needed to change to different constants later

        windmillPID = new ProfiledPIDController(0, 0, 0, windmillConstraints); //set to what we need later
        windmillFF = new ArmFeedforward(0, 0, 0); // Check ^^ (Also what in the world is Ks Kg and Kv)

        windmillMech2d = new Mechanism2d(60, 60); // Find the right size for this
        root = windmillMech2d.getRoot("WindmillRoot", 30, 30); // Create root at the center
        windmillLigament = root.append(new MechanismLigament2d("Windmill", 30, 0)); // Create ligament representing the windmill

        SmartDashboard.putData("Windmill Mechanism", windmillMech2d); // Add Mechanism2d to SmartDashboard

    }

    public void setVoltage(double voltage) {
        windmillIo.setVoltage(voltage);
    }

    public void stop() {
        windmillIo.stop();
    }
    

    public void setTargetPosition(double degrees) {
        windmillPID.setGoal(Units.degreesToRadians(degrees)); // basically this is telling the windmill "we wanna go here, get there"
        
        windmillIo.setVoltage(
            windmillPID.calculate(
                windmillInputs.absolutePositionRadians,
                Units.degreesToRadians(degrees))
            +   windmillFF.calculate(
                Units.degreesToRadians(degrees),
                windmillPID.getSetpoint().velocity));

                // Update the windmill ligament angle
        windmillLigament.setAngle(degrees);
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

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Update the Mechanism2d with the current state of the windmill
        SmartDashboard.putData("Windmill Mechanism", windmillMech2d);
    }


}
