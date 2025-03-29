package frc.robot.Interpolation;

import java.util.function.DoubleSupplier;

public class WindmillTable {
        
    
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> launcherMap = new InterpolatingTreeMap<>();
    
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> windmillMap = new InterpolatingTreeMap<>();
        private static final double PIGEON_OFFSET = 180;
    
    // key = distance from barge meters
    // value = angle setpoint
        static {    
            windmillMap.put(new InterpolatingDouble(1.44), new InterpolatingDouble(57.0));
            windmillMap.put(new InterpolatingDouble(1.55), new InterpolatingDouble(51.0));
            
                 }
    
        public DoubleSupplier getSetpointSupplier(double distance) {
            return () -> windmillMap.getInterpolated(new InterpolatingDouble(distance)).value;
        }
    
        public double getSetpoint(double distance) {
            return windmillMap.getInterpolated(new InterpolatingDouble(distance)).value;
        }
    }   

