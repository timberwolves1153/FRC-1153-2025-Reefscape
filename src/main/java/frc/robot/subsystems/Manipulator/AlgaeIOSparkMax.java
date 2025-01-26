package frc.robot.subsystems.Manipulator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeIOSparkMax implements AlgaeIO {

  private SparkMax wheelsC;
  private SparkMaxConfig config;

  public AlgaeIOSparkMax() {

    wheelsC = new SparkMax(46, MotorType.kBrushless);
    config = new SparkMaxConfig();
  }

  public void configMotors() {

    // all post-deprecation REV motor configs

    config.smartCurrentLimit(40);
    config.idleMode(IdleMode.kBrake);

    wheelsC.clearFaults();
    wheelsC.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override // matches names w/in algae
  public void updateInputs(AlgaeIOInputs inputs) {

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
}
