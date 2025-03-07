package frc.robot.subsystems.Manipulator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeIOSparkMax implements AlgaeIO {

  private SparkFlex launchMotor;
  private SparkMax holdMotor;
  private SparkMaxConfig config;
  private SparkMaxConfig launchConfig;

  public AlgaeIOSparkMax() {

    launchMotor = new SparkFlex(46, MotorType.kBrushless);
    holdMotor = new SparkMax(47, MotorType.kBrushless);
    config = new SparkMaxConfig();
    launchConfig = new SparkMaxConfig();
    configMotors();
  }

  public void configMotors() {

    // all post-deprecation REV motor configs

    config.smartCurrentLimit(60);

    launchConfig.smartCurrentLimit(60);

    launchConfig.inverted(true);

    launchMotor.clearFaults();
    launchMotor.configure(
        launchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    holdMotor.clearFaults();
    holdMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override // matches names w/in algae
  public void updateInputs(AlgaeIOInputs inputs) {

    inputs.outerAppliedVolts = launchMotor.getAppliedOutput() * 12;
    inputs.outerCurrentAmps = launchMotor.getOutputCurrent();

    inputs.innerAppliedVolts = holdMotor.getAppliedOutput() * holdMotor.getBusVoltage();
    inputs.innerCurrentAmps = holdMotor.getOutputCurrent();
  }

  @Override
  public void setVoltageLauncher(double volts) {
    launchMotor.setVoltage(volts);
  }

  @Override
  public void setVoltageHolding(double volts) {
    holdMotor.setVoltage(volts);
  }

  // ------------------------------------------
  @Override
  public void stopOuter() {
    launchMotor.setVoltage(0);
  }

  @Override
  public void stopInner() {
    holdMotor.setVoltage(0);
  }
}
