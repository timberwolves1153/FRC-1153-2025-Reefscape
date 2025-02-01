package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorIOTalonFX implements ElevatorIO {

  private TalonFX leftMotor = new TalonFX(41, "rio");
  private TalonFX rightMotor = new TalonFX(42, "rio");
  public DigitalInput magnetSwitch;

    // Status Signals
  private StatusSignal<Angle> position;
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Current> current;
  private StatusSignal<Temperature> temp;
  private StatusSignal<Voltage> followerAppliedVolts;
  private StatusSignal<Current> followerCurrent;
  private StatusSignal<Temperature> followerTemp;

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
    
    position = leftMotor.getPosition();
    velocity = leftMotor.getVelocity();
    appliedVolts = leftMotor.getMotorVoltage();
    current = leftMotor.getStatorCurrent();
    temp = leftMotor.getDeviceTemp();
    followerAppliedVolts = leftMotor.getMotorVoltage();
    followerCurrent = leftMotor.getStatorCurrent();
    followerTemp = leftMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, current, temp);
    ParentDevice.optimizeBusUtilizationForAll(leftMotor);
  }

  @Override
  public boolean isSwitchTriggered() {
    return !magnetSwitch.get();
  }

  @Override
  public void setVoltage(double volts) {
    if (isSwitchTriggered() && volts < 0) {
      leftMotor.setVoltage(0.25);
    } else {
      leftMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }
  }

  @Override
  public void stop() {
    leftMotor.setVoltage(0);
  }

  @Override
  public void updateInputs(ElevatorInputs elevatorInputs) {
    BaseStatusSignal.refreshAll(position,
        velocity, 
        appliedVolts, followerAppliedVolts,
        current, followerCurrent,
        temp, followerTemp);

    elevatorInputs.positionRots = position.getValueAsDouble();
    elevatorInputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    elevatorInputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    elevatorInputs.appliedVolts =
        new double[] {appliedVolts.getValueAsDouble(), followerAppliedVolts.getValueAsDouble()};
    elevatorInputs.currentAmps =
        new double[] {current.getValueAsDouble(), followerCurrent.getValueAsDouble()};
    elevatorInputs.tempCelsius = new double[] {temp.getValueAsDouble(), followerTemp.getValueAsDouble()};
    elevatorInputs.isSwitchTriggered = isSwitchTriggered();

    SmartDashboard.putBoolean("Magnet Switch", isSwitchTriggered());
  }

  /* Called by the SysIdRoutine */
  private void voltageDrive(MutVoltage voltage) {
    leftMotor.setVoltage(voltage.in(Volts));
  }
}
