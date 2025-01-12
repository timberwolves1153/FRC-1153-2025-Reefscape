package frc.robot.subsystems.drive.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class Elevator {
    
    public ElevatorIO elevatorIO;
    public ElevatorInputsAutoLogged elevatorInputs;
    public PIDController elevatorPID;
    public TrapezoidProfile.Constraints constraints;
    public ProfiledPIDController profiledPIDController;
    public ElevatorFeedforward elevatorFF;

    public Elevator(ElevatorIO elevatorIO) {

        Mechanism2d elevatorMech2d = new Mechanism2d(10.0, 39.0);
        MechanismRoot2d elevatorRoot2d= elevatorMech2d.getRoot("Elevator Root", 0, 0);
        MechanismLigament2d elevatorLig2d = elevatorRoot2d.append(
            new MechanismLigament2d("Elevator", 63.0, 90));

        elevatorInputs = new ElevatorInputsAutoLogged();
        this.elevatorIO = elevatorIO;

        constraints = new TrapezoidProfile.Constraints(0, 0);
        profiledPIDController = new ProfiledPIDController(0, 0, 0, constraints);
        elevatorFF = new ElevatorFeedforward(0, 0, 0);
        elevatorPID = new PIDController(0, 0, 0);

    }

    public void setVoltage(double volts){
        elevatorIO.setVoltage(volts);
    }

    public void stopVoltage(){
        elevatorIO.setVoltage(0);
    }

    public double inchesToEncoderTicks(double setpoint) {
        double encoderTicks = 42.0;
        double shaftRadius = Units.metersToInches(0.008);

        double distancePerRevolution = shaftRadius * 2 * Math.PI;
        double conversion = distancePerRevolution/encoderTicks;

        return setpoint/conversion;
    }

    public void setTargetHeight(double inches) {
        profiledPIDController.setGoal(inchesToEncoderTicks(inches));

        elevatorIO.setVoltage(
            profiledPIDController.calculate(inchesToEncoderTicks(elevatorInputs.heightInches)) + 
            elevatorFF.calculate(profiledPIDController.getSetpoint().velocity));
    }

    public void holdTargetHeight() {
        profiledPIDController.setGoal(inchesToEncoderTicks(elevatorInputs.heightInches));

        elevatorIO.setVoltage(
            profiledPIDController.calculate(inchesToEncoderTicks(elevatorInputs.heightInches)) + 
            elevatorFF.calculate(profiledPIDController.getSetpoint().velocity));
    }

    // @Override
    // public void periodic() {
    //     elevatorIO.updateInputs(elevatorInputs);
    //     Logger.processInputs("Elevator Inputs", elevatorInputs);
    //   //  Logger.recordOutput("Elevator Setpoint", profiledPIDController.getSetpoint());
    // }


}
