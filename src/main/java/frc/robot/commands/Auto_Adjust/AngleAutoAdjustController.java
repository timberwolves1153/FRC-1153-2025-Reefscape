package frc.robot.commands.Auto_Adjust;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;

public class AngleAutoAdjustController implements SwerveController {

  private static PIDController pid;
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  private Supplier<Double> measurment;
  private double omega;
  /**
   * @param getMeasurment Must be absolute.
   */
  public AngleAutoAdjustController(Supplier<Double> getMeasurment, double setPointDegrees) {

    pid = new PIDController(0.2, 0, 0);

    measurment = getMeasurment;
    pid.setSetpoint(setPointDegrees);
    pid.setTolerance(0.5);
    pid.enableContinuousInput(180, -180);
  }

  public AngleAutoAdjustController(Supplier<Double> getMeasurment) {
    this(getMeasurment, 165);
  }

  public ChassisSpeeds update() {

    omega = pid.calculate(measurment.get());
    chassisSpeeds.omegaRadiansPerSecond = omega;

    return chassisSpeeds;
  }

  public boolean getAtPoint() {
    return pid.atSetpoint();
  }

  public double getSetPoint() {
    return pid.getSetpoint();
  }

  public void setSetPoint(double setPointDrgrees) {
    pid.setSetpoint(setPointDrgrees);
  }

  public void updatePID(PIDController pidController) {
    pid = pidController;
  }
}
