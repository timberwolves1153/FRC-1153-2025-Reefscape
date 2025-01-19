package frc.robot.subsystems.Manipulator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class CoralIOSim implements CoralIO {
    private final DCMotorSim sim;
    private double appliedVolts = 0.0;
    
    public CoralIOSim(DCMotor motor, double reduction, double momentOfInertia) {
        sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, momentOfInertia, reduction), motor);
    }

    @Override
    public void updateInputs(CoralIOInputs inputs) {
        if(DriverStation.isDisabled()){
            setVoltage(0);
        }

        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double volts){
        appliedVolts = MathUtil.clamp(volts, -12, 12); //can be edited 
        sim.setInputVoltage(appliedVolts);
    }
    
    @Override
    public void stop(){
        setVoltage(0);
    }
}
