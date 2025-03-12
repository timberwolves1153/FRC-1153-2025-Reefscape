package frc.robot.subsystems.Climber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.Climber.ClimberIO.ClimberInputs;

public class ClimberIOSparkMax implements ClimberIO {

  private SparkMax climbMotor;
  private SparkClosedLoopController pid;

  public ClimberIOSparkMax() {

    climbMotor = new SparkMax(51, MotorType.kBrushless);
    pid = climbMotor.getClosedLoopController();

    climbMotor.getEncoder().setPosition(0);
    configMotor();
  }

  public void configMotor() {
    var config = new SparkMaxConfig();

    config.smartCurrentLimit(40);
    config.idleMode(IdleMode.kBrake);
    config.closedLoop.p(1);
    config.closedLoop.i(0);
    config.closedLoop.d(0);
    climbMotor.clearFaults();
    climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ClimberInputs climberInputs) {
    climberInputs.appliedVoltage = climbMotor.getAppliedOutput();
    climberInputs.climberCurrentAmps = climbMotor.getOutputCurrent();
    climberInputs.encoderCounts = climbMotor.getEncoder().getPosition();
  }

  @Override
  public void setVoltage(double volts) {
    climbMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    climbMotor.setVoltage(0);
  }

  @Override
  public void setPosition(double position) {
    pid.setReference(position, ControlType.kPosition);
  }

  @Override
  public void zeroClimb() {
    climbMotor.getEncoder().setPosition(0);
  }
}
