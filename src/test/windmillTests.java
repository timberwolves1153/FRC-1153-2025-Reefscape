import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.windmill.Windmill;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class WindmillTests {

  private static final double DELTA = 1e-2;
  private Windmill windmill;
  private DCMotorSim simMotor;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    windmill = new Windmill();
    simMotor = new DCMotorSim(DCMotor.getKrakenX60(1));
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    windmill.close();
  }

  @Test
  void testHold() {
    windmill.holdPosition();
    assertEquals(windmill.getAngle(), windmill.getAngle(), DELTA);
  }

  @Test
  void testLimitSwitch() {
    windmill.setTargetPosition(30);
    assertEquals(30, windmill.getAngle(), DELTA);
    windmill.setTargetPosition(90);
    assertEquals(90, windmill.getAngle(), DELTA);
    windmill.setTargetPosition(180);
    assertEquals(180, windmill.getAngle(), DELTA);
    windmill.setTargetPosition(270);
    assertEquals(270, windmill.getAngle(), DELTA);
    windmill.setTargetPosition(360);
    assertEquals(360, windmill.getAngle(), DELTA);

    // THESE CAN BE ADJUSTED FOR WHEN WE ACTULLY HAVE THE LIMIT SWITCHES
    // IE WHEN WE HAVE THE LIMIT SWITCHES, WE CAN MAKE SURE THAT THE ANGLE ISNT PAST THE LIMIT
    // SWITCH
  }

  @Test
  void testAngles() {
    windmill.setTargetPosition(30);
    assertEquals(30, windmill.getAngle(), DELTA);
  }

  @Test
  void testVolts() {
    windmill.setVoltage(6);
    assertEquals(6, windmill.getVoltage(), DELTA);
  }
}
