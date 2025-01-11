package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO{
    private ElevatorSim elevatorSim;
    private PIDController elevatorPID;

    public ElevatorIOSim() {
        
        elevatorSim = new ElevatorSim(
            DCMotor.getNEO(2), 
            12, 
            Units.lbsToKilograms(25.344), 
            3.0, 
            0, 
            55, 
            true, 
            0);

        elevatorPID = new PIDController(0.5, 0, 0);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        elevatorSim.update(0.02);

        inputs.positionInches = getElevatorHeight();
        
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