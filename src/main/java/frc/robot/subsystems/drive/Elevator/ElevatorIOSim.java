package frc.robot.subsystems.drive.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO{

    private ElevatorSim elevatorSim;
    private PIDController elevatorPID;

    public ElevatorIOSim()  {
        elevatorSim = new ElevatorSim(
            DCMotor.getFalcon500(2), 
            12, 
            Units.lbsToKilograms(17.966), 
            0.0, 
            Units.inchesToMeters(0), 
            Units.inchesToMeters(63.0), 
            true, 
            Units.inchesToMeters(0));

        elevatorPID = new PIDController(0, 0, 0);

    }

    @Override
    public void updateInputs(ElevatorInputs elevatorInputs) {
        elevatorSim.update(0.02);

        elevatorInputs.heightInches = getElevatorHeight();
    }

    @Override
    public void setVoltage(double volts) {
        elevatorSim.setInputVoltage(volts);
    }

    @Override
    public void stop() {
        elevatorSim.setInputVoltage(0);
    }

    public double getElevatorHeight() {
        return Units.metersToInches(elevatorSim.getPositionMeters());
    }


    
}
