package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorIOTalonFX implements ElevatorIO {

  private TalonFX leftMotor = new TalonFX(41, "rio");
  private TalonFX rightMotor = new TalonFX(42, "rio");
  private VoltageOut voltageRequest;
  private MotionMagicVoltage positionRequest;
  private final TrapezoidProfile trapezoidProfile;

  public DigitalInput magnetSwitch;

  private final StatusSignal<Current> leaderCurrentValue = leftMotor.getSupplyCurrent();
  private final StatusSignal<Voltage> leaderAppliedVolts = leftMotor.getMotorVoltage();
  private final StatusSignal<Angle> leaderPosition = leftMotor.getPosition();
  private final StatusSignal<Temperature> leaderTemp = leftMotor.getDeviceTemp();

  private final StatusSignal<Current> followerCurrentValue = rightMotor.getSupplyCurrent();
  private final StatusSignal<Voltage> followerAppliedVolts = rightMotor.getMotorVoltage();
  private final StatusSignal<Angle> followerPosition = rightMotor.getPosition();
  private final StatusSignal<Temperature> followerTemp = rightMotor.getDeviceTemp();

  public ElevatorIOTalonFX() {
    magnetSwitch = new DigitalInput(9);
    voltageRequest = new VoltageOut(0);
    positionRequest = new MotionMagicVoltage(0).withSlot(0);
    leftMotor.setPosition(0);

    trapezoidProfile =
        new TrapezoidProfile( // Units are rotations per second & rotations per seocond^2
            new TrapezoidProfile.Constraints(80, 160));

    config();
  }

  public void config() {
    var config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // in init function, set slot 0 gains
    // the values below are NOT correct and need to be adjusted to work with the windmill
    var slot0Configs = config.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    leftMotor.getConfigurator().apply(config);
    rightMotor.getConfigurator().apply(config);

    rightMotor.setControl(new Follower(41, true));

    // config.Feedback.SensorToMechanismRatio = 7.1429;

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leaderCurrentValue,
        leaderAppliedVolts,
        leaderPosition,
        leaderTemp,
        followerAppliedVolts,
        followerCurrentValue,
        followerPosition,
        followerTemp);

    leftMotor.optimizeBusUtilization();
    rightMotor.optimizeBusUtilization();
  }

  private boolean isSwitchTriggered() {
    return !magnetSwitch.get();
  }

  @Override
  public void setVoltage(Voltage volts) {
    if (isSwitchTriggered() && volts.baseUnitMagnitude() < 0) {
      leftMotor.setControl(voltageRequest.withOutput(0.25));
    } else {
      leftMotor.setControl(voltageRequest.withOutput(volts));
    }
  }

  @Override
  public void stop() {
    leftMotor.setControl(voltageRequest.withOutput(null));
  }

  @Override
  public void setTargetHeight(double rotations) {
    leftMotor.setControl(positionRequest.withPosition(rotations));
  }

  @Override
  public void updateInputs(ElevatorInputs elevatorInputs) {
    BaseStatusSignal.refreshAll(
        leaderCurrentValue, leaderAppliedVolts,
        leaderPosition, leaderTemp,
        followerAppliedVolts, followerCurrentValue,
        followerPosition, followerTemp);

    elevatorInputs.elevatorCurrentAmps = leaderCurrentValue.getValueAsDouble();
    elevatorInputs.elevatorCurrentAmps = followerCurrentValue.getValueAsDouble();

    elevatorInputs.getAppliedVolts = leaderAppliedVolts.getValueAsDouble();
    elevatorInputs.getAppliedVolts = followerAppliedVolts.getValueAsDouble();

    elevatorInputs.tempCelsius = leaderTemp.getValueAsDouble();
    elevatorInputs.tempCelsius = followerTemp.getValueAsDouble();
    elevatorInputs.isSwitchTriggered = isSwitchTriggered();
    elevatorInputs.leaderRotations = leaderPosition.getValueAsDouble();

    SmartDashboard.putBoolean("Magnet Switch", isSwitchTriggered());
  }
}
