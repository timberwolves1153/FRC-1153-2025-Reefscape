package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO{

    private TalonFX leftMotor, rightMotor;
    private VoltageOut voltageOut;
    private MotionMagicVoltage positionVoltage;

    // misusing type system here - these correspond to linear meters, NOT rotations
    private StatusSignal<Angle> position;
    private StatusSignal<AngularVelocity> velocity;
    private StatusSignal<Voltage> voltage;
    private StatusSignal<Current> statorCurrent;
    private StatusSignal<Current> supplyCurrent;
    private  StatusSignal<Temperature> temp;
    
    
    public ElevatorIOTalonFX() {

        leftMotor = new TalonFX(41, "rio");
        rightMotor = new TalonFX(42, "rio");
        
        voltageOut = new VoltageOut(0.0).withEnableFOC(false);
        positionVoltage = new MotionMagicVoltage(0.0).withEnableFOC(false);

        position = leftMotor.getPosition();
        velocity = leftMotor.getVelocity();
        voltage = leftMotor.getMotorVoltage();
        statorCurrent = leftMotor.getStatorCurrent();
        supplyCurrent = leftMotor.getSupplyCurrent();
        temp = leftMotor.getDeviceTemp();

        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.kG = 0.11591;
        config.Slot0.kS = 0.16898;
        config.Slot0.kV = 11.3;
        config.Slot0.kA = 0.0;
        config.Slot0.kP = 150.0;
        config.Slot0.kD = 17.53;

            
        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // ADJUST TO FIT OUR ROBOT 
        config.MotionMagic.MotionMagicAcceleration = 8.0;
        // Estimated from slightly less than motor free speed
        config.MotionMagic.MotionMagicCruiseVelocity =
            50.0 / (1 * 2 * Math.PI * 3);
        // THE "1" IS SUPPORSE TO BE GEAR RATIO AND THE "3" IS THE DRUM RADIUS IN METERS

        // Carriage position meters in direction of elevator
        config.Feedback.SensorToMechanismRatio =
            1 / (2 * Math.PI * 3);

        leftMotor.getConfigurator().apply(config);
        leftMotor.setPosition(0.0); // Assume we boot 0ed
        rightMotor.getConfigurator().apply(config);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, position, velocity, voltage, statorCurrent, supplyCurrent, temp);
        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();

    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        BaseStatusSignal.refreshAll(position, velocity, voltage, statorCurrent, supplyCurrent, temp);
        inputs.positionMeters = position.getValueAsDouble();
        inputs.velocityMetersPerSec = velocity.getValueAsDouble();
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
    }


    @Override
    public void setTargetHeight(final double meters) {
      leftMotor.setControl(positionVoltage.withPosition(meters));
    }
  
    @Override
    public void setVoltage(final double voltage) {
      leftMotor.setControl(voltageOut.withOutput(voltage));
    }
  
    @Override
    public void zeroElevatorEncoder() {
      leftMotor.setPosition(0);
    }   

}
