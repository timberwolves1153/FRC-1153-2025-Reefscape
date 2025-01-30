package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorIOTalonFX implements ElevatorIO {

  private TalonFX leftMotor = new TalonFX(41, "rio");
  private TalonFX rightMotor = new TalonFX(42, "rio");

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

    config.Feedback.SensorToMechanismRatio = 12 / (2 * Math.PI * Units.inchesToMeters(1.7567));

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

  @Override
  public boolean isSwitchTriggered() {
    return !magnetSwitch.get();
  }

  public void moveElevator(double volts) {
    double adjustedVolts = volts;
    if (isSwitchTriggered() && adjustedVolts < 0) {
      adjustedVolts = 0;
    } else {
      adjustedVolts = volts;
    }
  }

  @Override
  public void setVoltage(double volts) {
    leftMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
  }

  @Override
  public void stop() {
    leftMotor.setVoltage(0);
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

    elevatorInputs.heightMeters = leaderPosition.getValueAsDouble();

    elevatorInputs.getAppliedVolts = leaderAppliedVolts.getValueAsDouble();
    elevatorInputs.getAppliedVolts = followerAppliedVolts.getValueAsDouble();

    elevatorInputs.tempCelsius = leaderTemp.getValueAsDouble();
    elevatorInputs.tempCelsius = followerTemp.getValueAsDouble();

    SmartDashboard.putBoolean("Magnet Switch", isSwitchTriggered());
  }
}
