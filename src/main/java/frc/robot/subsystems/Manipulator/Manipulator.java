package frc.robot.subsystems.Manipulator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Manipulator extends SubsystemBase {
    private SparkMax wheels;
    private SparkMaxConfig config;

    public Manipulator() {

        wheels = new SparkMax(10, MotorType.kBrushless);
        config = new SparkMaxConfig();
    }

    public void configMotors(){

        config.smartCurrentLimit(40);
        wheels.clearFaults();
        config.idleMode(IdleMode.kBrake);
        wheels.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setWheelsVolts(double volts) {
        wheels.setVoltage(volts);
    }

    public void intakeCoral(){
        wheels.setVoltage(-4);
    }

    public void outtakeCoral(){
        wheels.setVoltage(4);
    }

    public void stopCoral(){
        wheels.setVoltage(0);
    }
}
