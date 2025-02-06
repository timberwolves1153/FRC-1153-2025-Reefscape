package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.windmill.Windmill;
import frc.robot.subsystems.windmill.WindmillIOFX;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class WindmillTest {

  static final double DELTA = 1e-2; // acceptable deviation range
  Windmill windmill;
  TalonFX talonFx;
  TalonFXSimState fx_sim;
  CANcoder cancoder;
  CANcoderSimState cancoder_sim;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

    /* create the TalonFX */
    talonFx = new TalonFX(43);
    fx_sim = talonFx.getSimState();

    cancoder = new CANcoder(44);
    cancoder_sim = cancoder.getSimState();

    windmill = new Windmill(new WindmillIOFX());

    /* enable the robot */
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    /* delay ~100ms so the devices can start up and enable */
    Timer.delay(0.100);
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    windmill.close(); // destroy our intake object
  }

  @Test // marks this method as a test
  void motorPower() {
    double desiredVolts = 4;
    fx_sim.setSupplyVoltage(RobotController.getBatteryVoltage());

    StatusSignal<Double> dutyCycle = talonFx.getDutyCycle();

    /* wait for a fresh duty cycle signal */
    dutyCycle.waitForUpdate(0.100);
    /* verify that the motor output is zero */
    assertEquals(dutyCycle.getValue(), 0.0, DELTA);

    // Set the windmill voltage to the desired volts
    windmill.setVoltage(desiredVolts);
    /* wait for the control to apply */
    Timer.delay(0.020);
    dutyCycle.waitForUpdate(0.100);

    assertEquals(
        RobotController.getBatteryVoltage() / desiredVolts,
        fx_sim.getMotorVoltage(),
        DELTA); // make sure that the value set to the motor is equal to the desired voltage

    windmill.stop();
    /* wait for a fresh duty cycle signal */
    dutyCycle.waitForUpdate(0.100);
    /* verify that the motor output is zero */
    assertEquals(fx_sim.getMotorVoltage(), 0.0, DELTA);
  }

  @Test // Ensure that the motor has postive voltage when setting the algae desired position postive
  void AlgaePostive() {
    // double desiredVolts = 4;
    fx_sim.setSupplyVoltage(RobotController.getBatteryVoltage());

    StatusSignal<Double> dutyCycle = talonFx.getDutyCycle();

    /* wait for a fresh duty cycle signal */
    dutyCycle.waitForUpdate(0.100);
    /* verify that the motor output is zero to start*/
    assertEquals(dutyCycle.getValue(), 0.0, DELTA);

    // Set the windmill voltage to the desired volts
    windmill.setTargetPosition(90);
    /* wait for the control to apply */
    Timer.delay(0.020);
    dutyCycle.waitForUpdate(0.100);

    System.out.println("Current Position : " + cancoder.getPosition());
    System.out.println("Target Position : " + windmill.getTargetPosition());
    System.out.println("Motor Voltage : " + fx_sim.getMotorVoltage());
    System.out.println("Motor Greater than 0 : " + (fx_sim.getMotorVoltage() > 0));

    assertTrue(
        fx_sim.getMotorVoltage()
            > 0); // make sure that the value set to the motor is postive when the goal is higher

    windmill.stop();
    /* wait for a fresh duty cycle signal */
    dutyCycle.waitForUpdate(0.100);
    /* wait for the control to apply */
    Timer.delay(0.020);
    /* verify that the motor output is zero */
    assertEquals(fx_sim.getMotorVoltage(), 0.0, DELTA);
  }

  @Test // Ensure that the motor has postive voltage when setting the algae desired position postive
  void AlgaeNegative() {
    // double desiredVolts = 4;
    fx_sim.setSupplyVoltage(RobotController.getBatteryVoltage());

    StatusSignal<Double> dutyCycle = talonFx.getDutyCycle();

    /* wait for a fresh duty cycle signal */
    dutyCycle.waitForUpdate(0.100);
    /* verify that the motor output is zero to start*/
    assertEquals(dutyCycle.getValue(), 0.0, DELTA);

    // Set the windmill voltage to the desired volts
    windmill.setTargetPosition(-90);
    /* wait for the control to apply */
    Timer.delay(0.020);
    dutyCycle.waitForUpdate(0.100);

    System.out.println("Current Position : " + cancoder.getPosition());
    System.out.println("Target Position : " + windmill.getTargetPosition());
    System.out.println("Motor Voltage : " + fx_sim.getMotorVoltage());
    System.out.println("Motor Less than 0 : " + (fx_sim.getMotorVoltage() < 0));

    assertTrue(
        fx_sim.getMotorVoltage()
            < 0); // make sure that the value set to the motor is postive when the goal is higher

    windmill.stop();
    /* wait for a fresh duty cycle signal */
    dutyCycle.waitForUpdate(0.100);
    /* wait for the control to apply */
    Timer.delay(0.020);
    /* verify that the motor output is zero */
    assertEquals(fx_sim.getMotorVoltage(), 0.0, DELTA);
  }
}
