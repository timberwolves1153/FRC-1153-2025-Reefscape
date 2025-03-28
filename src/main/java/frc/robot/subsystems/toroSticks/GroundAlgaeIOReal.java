package frc.robot.subsystems.toroSticks;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class GroundAlgaeIOReal implements GroundAlgaeIO {

  private SparkMax pivotMotor;
  private SparkClosedLoopController pivotPID;
  private RelativeEncoder pivotEncoder;
  private TalonFX rollerMotor;
  private VoltageOut voltageRequest;

  public GroundAlgaeIOReal() {

    pivotMotor = new SparkMax(48, MotorType.kBrushless);
    pivotPID = pivotMotor.getClosedLoopController();
    pivotEncoder = pivotMotor.getEncoder();
    rollerMotor = new TalonFX(49, "rio");
    voltageRequest = new VoltageOut(0);
    pivotEncoder.setPosition(0);
    configMotors();
  }

  @Override
  public void updateInputs(GroundAlgaeInputs inputs) {
      inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() *12;
      inputs.pivotCurrentAmps = pivotMotor.getOutputCurrent();
      inputs.pivotRotations = pivotEncoder.getPosition();
      inputs.rollerAppliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
      inputs.rollerCurrentAmps = rollerMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void pivotDown() {
    pivotMotor.setVoltage(3);
  }

  @Override
  public void pivotUp() {
    pivotMotor.setVoltage(-3);
  }

  @Override
  public void pivotStop() {
    pivotMotor.setVoltage(0);
  }

  @Override
  public void intake() {
    rollerMotor.setControl(voltageRequest.withOutput(6));
  }

  @Override
  public void outtake() {
    rollerMotor.setControl(voltageRequest.withOutput(-6));
  }

  @Override
  public void stopRollers() {
    rollerMotor.setControl(voltageRequest.withOutput(0));
  }

  @Override
  public void setPivotPostion(double rotations) {
    pivotPID.setReference(rotations, ControlType.kPosition);
  }

  @Override
  public void resetPivotEncoder() {
    pivotEncoder.setPosition(0);
  }

  public void configMotors() {
    var sparkConfig = new SparkMaxConfig();

    sparkConfig.smartCurrentLimit(40);
    sparkConfig.idleMode(IdleMode.kBrake);
    sparkConfig.closedLoop.p(1);
    sparkConfig.closedLoop.i(0);
    sparkConfig.closedLoop.d(0);
    pivotMotor.clearFaults();
    pivotMotor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var talonConfig = new TalonFXConfiguration();

    talonConfig.CurrentLimits.StatorCurrentLimit = 40;
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rollerMotor.getConfigurator().apply(talonConfig);

  }
}
