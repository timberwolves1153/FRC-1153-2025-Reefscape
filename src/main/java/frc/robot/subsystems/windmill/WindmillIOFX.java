package frc.robot.subsystems.windmill;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class WindmillIOFX implements WindmillIO {

  private TalonFX windmillMotor;
  private CANcoder encoder;
  private Windmill windmill;

  private final StatusSignal<Angle> windmillPosition;
  private final StatusSignal<Voltage> windmillAppliedVolts;
  private final StatusSignal<Current> windmillCurrent;

  public WindmillIOFX() {

    windmillMotor = new TalonFX(43, "rio"); // double CHECK
    encoder = new CANcoder(44); // double CHECK

    windmillPosition = windmillMotor.getPosition();
    windmillAppliedVolts = windmillMotor.getMotorVoltage();
    windmillCurrent = windmillMotor.getSupplyCurrent();

    configMotors();
  }

  public void configMotors() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    windmillMotor.getConfigurator().apply(config);

    windmillMotor.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, windmillPosition, windmillAppliedVolts, windmillCurrent);
  }

  @Override
  public void updateInputs(WindmillInputs inputs) {
    inputs.absolutePosition =
        Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble());
    inputs.absolutePositionRadians = getPositionRadians();
    inputs.absolutePositionDegrees = getPositionDegrees();
    inputs.appliedVolts = windmillAppliedVolts.getValueAsDouble();
    inputs.current = windmillCurrent.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    windmillMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    windmillMotor.setVoltage(0);
  }

  /**
   * The position of the windmill adjusted to the standard convention of tracking algae maniupulator
   * angle.
   *
   * @return
   */
  private double getCalculatedPosition() {
    return encoder.getAbsolutePosition().getValueAsDouble();
  }

  private double getAbsolutePositionRotations() {
    return encoder.getAbsolutePosition().getValueAsDouble();
  }

  private double getPositionRadians() {
    return Units.rotationsToRadians(getAbsolutePositionRotations());
  }

  private double getPositionDegrees() {
    return Units.rotationsToDegrees(getAbsolutePositionRotations());
  }

  public void close() {
    windmillMotor.close();
    encoder.close();
  }
}
