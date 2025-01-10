package frc.robot.subsystems.Windmill;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WindmillIOSim implements WindmillIO {
    
    private SingleJointedArmSim windmillSim;


    public WindmillIOSim() {

        windmillSim = new SingleJointedArmSim(DCMotor.getNEO(1), 0, 0, 0, 0, 0, false, 0, null); //GET ALL OF THESE FROM HE CAD WHEN IT IS DONE
    }
}
