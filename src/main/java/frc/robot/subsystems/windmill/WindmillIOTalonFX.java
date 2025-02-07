package frc.robot.subsystems.windmill;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class WindmillIOTalonFX implements WindmillIO {

  private TalonFX windmillMotor;
  private CANcoder encoder;
  private Windmill windmill;
  private VoltageOut voltageRequest;
  private MotionMagicVoltage positionRequest;

  private final StatusSignal<Angle> windmillPosition;
  private final StatusSignal<Voltage> windmillAppliedVolts;
  private final StatusSignal<Current> windmillCurrent;

  // Trapezoid profile with max velocity 80 rps, max accel 160 rps/s
  private final TrapezoidProfile trapezoidProfile;

  private final double WINDMILL_OFFSET_DEGREES = 56.68; // offset is in degrees, 0.1578 rotations
  private final double WINDMILL_OFFSET_ROTS = -0.1578;

  public WindmillIOTalonFX() {

    windmillMotor = new TalonFX(43, "rio"); // double CHECK
    encoder = new CANcoder(44); // double CHECK
    voltageRequest = new VoltageOut(0);
    positionRequest = new MotionMagicVoltage(0).withSlot(0);

    trapezoidProfile =
        new TrapezoidProfile( // Units are rotations per second & rotations per seocond^2
            new TrapezoidProfile.Constraints(80, 160));

    windmillPosition = windmillMotor.getPosition();
    windmillAppliedVolts = windmillMotor.getMotorVoltage();
    windmillCurrent = windmillMotor.getSupplyCurrent();

    configMotors();
    var encoderConfig = new CANcoderConfiguration();
    var magnetSensorConfigs = new MagnetSensorConfigs();
    magnetSensorConfigs
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(WINDMILL_OFFSET_ROTS);
    encoderConfig.withMagnetSensor(magnetSensorConfigs);
    encoder.getConfigurator().apply(encoderConfig);
  }

  public void configMotors() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // in init function, set slot 0 gains
    // the values below are NOT correct and need to be adjusted to work with the windmill
    var slot0Configs = config.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.25; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 55; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.SensorToMechanismRatio = 1;

    windmillMotor.getConfigurator().apply(config);

    windmillMotor.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, windmillPosition, windmillAppliedVolts, windmillCurrent);
  }

  @Override
  public void updateInputs(WindmillInputs inputs) {
    inputs.rotations = encoder.getAbsolutePosition().getValueAsDouble();
    inputs.appliedVolts = windmillAppliedVolts.getValueAsDouble();
    inputs.currentAmps = windmillCurrent.getValueAsDouble();
    inputs.tempCelsius = windmillMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setVoltage(Voltage volts) {
    windmillMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    windmillMotor.setControl(voltageRequest.withOutput(0));
  }

  @Override
  public void setTargetPosition(double rotations) {
    windmillMotor.setControl(positionRequest.withPosition(rotations));
  }

  public void close() {
    stop();
    windmillMotor.close();
    encoder.close();
  }
}
