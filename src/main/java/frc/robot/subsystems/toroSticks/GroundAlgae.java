package frc.robot.subsystems.toroSticks;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class GroundAlgae extends SubsystemBase {
  private final GroundAlgaeIO groundAlgaeIO;
  public GroundAlgaeInputsAutoLogged inputs;

  public GroundAlgae(GroundAlgaeIO io) {
    this.groundAlgaeIO = io;
    inputs = new GroundAlgaeInputsAutoLogged();
  }

  @Override
  public void periodic() {
    groundAlgaeIO.updateInputs(inputs);
    Logger.processInputs("Ground Algae", inputs);

    SmartDashboard.putNumber("ground rotations", inputs.pivotRotations);
  }

  public void pivotUp() {
    groundAlgaeIO.pivotUp();
  }

  public void pivotDown() {
    groundAlgaeIO.pivotDown();
  }

  public void pivotStop() {
    groundAlgaeIO.pivotStop();
  }

  public void intake() {
    groundAlgaeIO.intake();
  }

  public void outtake() {
    groundAlgaeIO.outtake();
  }

  public void stopRollers() {
    groundAlgaeIO.stopRollers();
  }

  public void deploy() {
    groundAlgaeIO.deploy();
  }

  public void stow() {
    groundAlgaeIO.stow();
  }

  public void setPivotPostion(double rotations) {
    groundAlgaeIO.setPivotPostion(rotations);
    ;
  }

  public void resetPivotEncoder() {
    groundAlgaeIO.resetPivotEncoder();
  }
}
