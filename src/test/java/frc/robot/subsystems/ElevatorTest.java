package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ElevatorTest {

  static final double DELTA = 1e-2; // acceptable deviation range
  Elevator elevator;
  TalonFX talonFx;
  TalonFXSimState fx_sim;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

    /* create the TalonFX */
    // talonFx = new TalonFX(41);
    // rightTalonFx = new TalonFX(42);
    // fx_sim = talonFx.getSimState();

    // cancoder = new CANcoder(44);
    // cancoder_sim = cancoder.getSimState();

    elevator = new Elevator(new ElevatorIOTalonFX());

    /* enable the robot */
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    /* delay ~100ms so the devices can start up and enable */
    Timer.delay(0.100);
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    // elevator.close(); // destroy our intake object
  }

  @Test // marks this method as a test
  void inchestoRotationsCalc() {
    double rots = elevator.inchesToRotations(5.5);
    System.out.println("Rotations " + rots);
    /* verify that the rotation output */
    assertEquals(7.14, rots, DELTA);

    rots = elevator.inchesToRotations(13.86);
    System.out.println("Rotations " + rots);
    /* verify that the rotation output */
    assertEquals(18, rots, DELTA);
  }
}
