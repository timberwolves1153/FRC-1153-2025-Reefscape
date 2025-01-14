package frc.robot.subsystems.elevator;


import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    
    public ElevatorInputsAutoLogged inputs;
    public ElevatorIO io;

    
    private PIDController elevatorPID;
    private ElevatorFeedforward elevatorFF;
    private TrapezoidProfile.Constraints elevatorConstraints;
    private ProfiledPIDController profiledPID;

    private Mechanism2d mech2d;
    
    private MechanismRoot2d root2d;
    private MechanismLigament2d elevatorLig2d;

    public Elevator(ElevatorIO io) {

        this.io = io;
        inputs = new ElevatorInputsAutoLogged();


         elevatorConstraints =
        new TrapezoidProfile.Constraints(60.0, 40.0);

        profiledPID =
        new ProfiledPIDController(0.25 / 7, 0.0, 0.0139 / 2, elevatorConstraints);
        elevatorPID = new PIDController(0.5, 0, 0);
        elevatorFF = new ElevatorFeedforward(1.0e-2, 0.11, 0.01);
        
        
        mech2d = new Mechanism2d(2, 3);
        root2d = mech2d.getRoot("Elevator Root", 0, Units.inchesToMeters(10));
        elevatorLig2d =
            root2d.append(
                new MechanismLigament2d(
                    "Elevator",
                    20,
                    0,
                    15,
                    new Color8Bit(Color.kPurple)));
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        io.stop();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        

        Logger.recordOutput("velocity to goal", profiledPID.getSetpoint().velocity);
        
    Logger
        .recordOutput(
            "Elevator Goal", profiledPID.getGoal().position);
    Logger
        .recordOutput(
            "Elevator Setpoint", profiledPID.getSetpoint().position);

        updateMech2d();

        //SmartDashboard.putData("mech2d", mech2d);

       // Logger.recordOutput("Mech 2D", mech2d);
        
        
        
    }



    public void updateMech2d() {
        elevatorLig2d.setLength(Units.inchesToMeters(inputs.positionInches)+ 20);
      }
    

    public double inchesToTicks(double setpoint) {
        double ticksPerRev = 42;
        double shaftRadius = 0.0625;
        double distanceperRev = shaftRadius * Math.PI * 2;
        double conversionFactor = distanceperRev / ticksPerRev;

        return setpoint / conversionFactor;
    }

    public void setHeight(double inches) {
        profiledPID.setGoal(inchesToTicks(inches));
        io.setVoltage(
            profiledPID.calculate(inchesToTicks(inputs.positionInches)) 
            + elevatorFF.calculate(profiledPID.getSetpoint().velocity));
    }

    public void holdPosition() {
        profiledPID.setGoal(inchesToTicks(inputs.positionInches));

        io.setVoltage(
            profiledPID.calculate(inchesToTicks(inputs.positionInches)) 
            + elevatorFF.calculate(profiledPID.getSetpoint().velocity));
    }


}
