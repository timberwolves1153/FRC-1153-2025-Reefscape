package frc.robot.subsystems.Manipulator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class CoralIOSparkMax implements CoralIO {

  private SparkMax wheelsC;
  private SparkMaxConfig config;

  private final DoubleSolenoid doubleSolenoid;

  public CoralIOSparkMax() {

    wheelsC = new SparkMax(10, MotorType.kBrushless);
    config = new SparkMaxConfig();
    doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  }

  public void configMotors() {

    // all post-deprecation REV motor configs

    config.smartCurrentLimit(40);
    config.idleMode(IdleMode.kBrake);

    wheelsC.clearFaults();
    wheelsC.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override // matches names w/in coralio
  public void updateInputs(CoralIOInputs inputs) {

    inputs.appliedVolts = wheelsC.getAppliedOutput() * wheelsC.getBusVoltage();
    inputs.currentAmps = wheelsC.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    wheelsC.setVoltage(volts);
  }
  // ------------------------------------------
  @Override
  public void stop() {
    wheelsC.setVoltage(0);
  }
  // ------------------------------------------
  @Override
  public void setSolenoid(DoubleSolenoid.Value value) {
    doubleSolenoid.set(value);
  }
}
