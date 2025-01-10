package frc.robot.subsystems.Windmill;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;


public class WindmillIOSparkMax implements WindmillIO{
    
    private TalonFX windmillMotor;
    private CANcoder encoder;

    public WindmillIOSparkMax() {

        windmillMotor = new TalonFX(51, "rio"); //double CHECK
        encoder = new CANcoder(52); //double CHECK


        configMotors();


    }

    @Override
    public void updateInputs(WindmillInputs inputs) {
        inputs.absolutePosition = Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble());
        inputs.absolutePositionRadians = encoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        inputs.appliedVolts = windmillMotor.get(); // FIND THE APPLIED VOLTS STSTSTSTSTRSTATTATATATATTTT

    }

    public void configMotors(){
        // WE NEED TO FIND THE IMPLEMENTATIONS OF THIS CODE AND ADAPT IT TO DO WHAT WE WANT IT TO DO
    }

    @Override
    public void setVoltage(double volts) {
        windmillMotor.setVoltage(volts);
    }

    @Override
    public void stop() {
        windmillMotor.setVoltage(0);
    }

    public double getAbsolutePosition() {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getPositionRadians() {
        return Units.rotationsToRadians(getAbsolutePosition());
    }

    public double getPositionDegrees() {
        return Units.rotationsToDegrees(getAbsolutePosition());
    }
}
