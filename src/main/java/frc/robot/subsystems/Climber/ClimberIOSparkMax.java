package frc.robot.subsystems.Climber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.Climber.ClimberIO.ClimberInputs;

public class ClimberIOSparkMax implements ClimberIO {

  private SparkMax climbMotor;

  public ClimberIOSparkMax() {

    climbMotor = new SparkMax(0, MotorType.kBrushless);

    configMotor();
    ;
  }

  public void configMotor() {
    var config = new SparkMaxConfig();

    config.smartCurrentLimit(40);
    config.idleMode(IdleMode.kBrake);
    climbMotor.clearFaults();
    climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ClimberInputs climberInputs) {
    climberInputs.appliedVoltage = climbMotor.getAppliedOutput();
    climberInputs.climberCurrentAmps = climbMotor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    climbMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    climbMotor.setVoltage(0);
  }
}
