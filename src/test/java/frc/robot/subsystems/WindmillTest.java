package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
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

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

    /* create the TalonFX */
    talonFx = new TalonFX(43);
    fx_sim = talonFx.getSimState();
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
    // System.out.println("DutyCycle val: " + dutyCycle.getValue());
    /* wait for a fresh duty cycle signal */
    // dutyCycle.waitForUpdate(0.100);

    assertEquals(
        RobotController.getBatteryVoltage() / desiredVolts,
        fx_sim.getMotorVoltage(),
        DELTA); // make sure that the value set to the motor is equal to the desired voltage
  }
}
