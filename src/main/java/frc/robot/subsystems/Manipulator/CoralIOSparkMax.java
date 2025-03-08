package frc.robot.subsystems.Manipulator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class CoralIOSparkMax implements CoralIO {

  private SparkMax coralMotor;
  private SparkMaxConfig config;

  // private final DoubleSolenoid doubleSolenoid;

  public CoralIOSparkMax() {

    coralMotor = new SparkMax(45, MotorType.kBrushless);
    config = new SparkMaxConfig();
    // doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    // doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void configMotors() {

    // all post-deprecation REV motor configs

    config.smartCurrentLimit(40);
    config.idleMode(IdleMode.kBrake);

    coralMotor.clearFaults();
    coralMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override // matches names w/in coralio
  public void updateInputs(CoralIOInputs inputs) {

    inputs.appliedVolts = coralMotor.getAppliedOutput() * coralMotor.getBusVoltage();
    inputs.currentAmps = coralMotor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    coralMotor.setVoltage(volts);
  }
  // ------------------------------------------
  @Override
  public void stop() {
    coralMotor.setVoltage(0);
  }
  // ------------------------------------------
  // @Override
  // public void setSolenoid() {
  //   doubleSolenoid.toggle();
  // }

  // @Override
  // public void setSolenoidState(Value position) {
  //   doubleSolenoid.set(position);
  // }

  // @Override
  // public Value getSolenoidState() {
  //   return doubleSolenoid.get();
  // }
}
