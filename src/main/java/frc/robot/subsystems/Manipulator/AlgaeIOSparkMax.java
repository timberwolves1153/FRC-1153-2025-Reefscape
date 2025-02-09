package frc.robot.subsystems.Manipulator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeIOSparkMax implements AlgaeIO {

  private SparkFlex outerWheelsC;
  private SparkMax innerWheelsC;
  private SparkMaxConfig config;

  public AlgaeIOSparkMax() {

    outerWheelsC = new SparkFlex(46,MotorType.kBrushless);
    innerWheelsC = new SparkMax(47, MotorType.kBrushless);
    config = new SparkMaxConfig();
  }

  public void configMotors() {

    // all post-deprecation REV motor configs

    config.smartCurrentLimit(40);
    config.idleMode(IdleMode.kBrake);

    outerWheelsC.clearFaults();
    outerWheelsC.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    innerWheelsC.clearFaults();
    innerWheelsC.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override // matches names w/in algae
  public void updateInputs(AlgaeIOInputs inputs) {

    inputs.outerAppliedVolts = outerWheelsC.getAppliedOutput() * outerWheelsC.getBusVoltage();
    inputs.outerCurrentAmps = outerWheelsC.getOutputCurrent();

    inputs.innerAppliedVolts = innerWheelsC.getAppliedOutput() * innerWheelsC.getBusVoltage();
    inputs.innerCurrentAmps = innerWheelsC.getOutputCurrent();
  }

  @Override
  public void setVoltageOuter(double volts) {
    outerWheelsC.setVoltage(volts);
  }

  @Override
  public void setVoltageInner(double volts) {
    outerWheelsC.setVoltage(volts);
  }

  // ------------------------------------------
  @Override
  public void stopOuter() {
    outerWheelsC.setVoltage(0);
  }

  @Override
  public void stopInner() {
    innerWheelsC.setVoltage(0);
  }
}
