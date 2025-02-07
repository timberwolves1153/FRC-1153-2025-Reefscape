package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefHeight;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ConstantsTest {

  static final double DELTA = 1e-2; // acceptable deviation range

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

    /* enable the robot */
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    /* delay ~100ms so the devices can start up and enable */
    Timer.delay(0.100);
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    // destroy our intake object
  }

  @Test // marks this method as a test
  void fieldTest() {
    assertEquals(22, FieldConstants.aprilTagCount);
    assertEquals(6, FieldConstants.Reef.centerFaces.length);
    assertEquals(12, FieldConstants.Reef.branchPositions.size());
    assertEquals(3.70, FieldConstants.Reef.branchPositions.get(0).get(ReefHeight.L2).getX(), DELTA);
  }
}
