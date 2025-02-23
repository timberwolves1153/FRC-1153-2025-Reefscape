package frc.robot.commands.Auto_Adjust;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface SwerveController {

  ChassisSpeeds update();
}
