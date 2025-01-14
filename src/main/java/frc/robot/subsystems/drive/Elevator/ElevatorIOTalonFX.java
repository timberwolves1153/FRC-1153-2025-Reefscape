package frc.robot.subsystems.drive.Elevator;

import org.ejml.equation.Variable;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;

public class ElevatorIOTalonFX implements ElevatorIO{
    
    private TalonFX leftMotor, rightMotor;
    private double elevatorEncoder;

    private final StatusSignal<Current> leaderCurrentValue = leftMotor.getSupplyCurrent(); 
    private final StatusSignal<Voltage> leaderAppliedVolts = leftMotor.getMotorVoltage();
    private final StatusSignal<Angle> leaderPosition = leftMotor.getPosition();

    private final StatusSignal<Current> followerCurrentValue = leftMotor.getSupplyCurrent(); 
    private final StatusSignal<Voltage> followerAppliedVolts = leftMotor.getMotorVoltage();
    private final StatusSignal<Angle> followerPosition = leftMotor.getPosition();

    public ElevatorIOTalonFX() {

        leftMotor = new TalonFX(41, "rio");
        rightMotor = new TalonFX(42, "rio");
        
        elevatorEncoder = leftMotor.getPosition().getValueAsDouble();

        config();


    }
    
    public void config() {
        var config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);

        rightMotor.setControl(new Follower(41, true));

        BaseStatusSignal.setUpdateFrequencyForAll( 
            0.0,
            leaderCurrentValue, leaderAppliedVolts, 
            leaderPosition, followerAppliedVolts, 
            followerCurrentValue, followerPosition);

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorInputs elevatorInputs) {
        BaseStatusSignal.refreshAll(
        leaderCurrentValue, leaderAppliedVolts,
        leaderPosition, followerAppliedVolts, 
        followerCurrentValue, followerPosition);

        //elevatorInputs.heightInches = elevatorEncoder.getEncoder();
        elevatorInputs.elevatorCurrentAmps = leaderCurrentValue.getValueAsDouble();
        elevatorInputs.elevatorCurrentAmps = followerCurrentValue.getValueAsDouble();

        elevatorInputs.getVoltageOut = leaderAppliedVolts.getValueAsDouble();
        elevatorInputs.getVoltageOut = followerAppliedVolts.getValueAsDouble();

    }

    @Override
    public void setVoltage(double volts) {
        leftMotor.setVoltage(volts);
    }

    @Override
    public void stop() {
        leftMotor.setVoltage(0);
    }





}


