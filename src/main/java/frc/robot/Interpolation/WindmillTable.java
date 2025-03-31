package frc.robot.Interpolation;

import java.util.function.DoubleSupplier;

public class WindmillTable {

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> launcherMap =
      new InterpolatingTreeMap<>();

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> windmillMap =
      new InterpolatingTreeMap<>();
  private static final double PIGEON_OFFSET = 180;

  // key = distance from barge meters
  // value = angle setpoint
  static {
    // windmillMap.put(new InterpolatingDouble(2.103), new InterpolatingDouble(-178.769));
    // windmillMap.put(new InterpolatingDouble(2.203), new InterpolatingDouble(-175.605));
    // windmillMap.put(new InterpolatingDouble(2.364), new InterpolatingDouble(-177.363));
    // windmillMap.put(new InterpolatingDouble(1.695), new InterpolatingDouble(-165.234));
    // windmillMap.put(new InterpolatingDouble(1.609), new InterpolatingDouble(-165.234));
    // windmillMap.put(new InterpolatingDouble(1.695), new InterpolatingDouble(-165.234));
    // windmillMap.put(new InterpolatingDouble(1.695), new InterpolatingDouble(-165.234));
    windmillMap.put(new InterpolatingDouble(1.037), new InterpolatingDouble(-154.775));
    windmillMap.put(new InterpolatingDouble(1.912), new InterpolatingDouble(-175.429));
    windmillMap.put(new InterpolatingDouble(2.764), new InterpolatingDouble(-175.429));

    launcherMap.put(new InterpolatingDouble(1.0), new InterpolatingDouble(4.0));
    launcherMap.put(new InterpolatingDouble(1.037), new InterpolatingDouble(5.9));
    launcherMap.put(new InterpolatingDouble(1.912), new InterpolatingDouble(8.0));
    launcherMap.put(new InterpolatingDouble(2.5), new InterpolatingDouble(12.0));
    launcherMap.put(new InterpolatingDouble(2.764), new InterpolatingDouble(12.0));
  }

  public DoubleSupplier getSetpointSupplier(double distance) {
    return () -> windmillMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public double getSetpoint(double distance) {
    return windmillMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }
}
