package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;

public class Superstructure extends SubsystemBase {
    
    public enum Goal {
        STOW,
        L1,
        L2,
        L3,
    }

    private Goal currentGoal = Goal.STOW;
    private Goal desiredGoal = Goal.STOW;
    private Goal lastGoal = Goal.STOW;

    private Elevator elevator;

    private Timer goalTimer = new Timer();

    public Superstructure() {
        this.elevator = elevator;

    }
    @Override
    public void periodic() {

        if (DriverStation.isDisabled()) {
      setDefaultCommand(setGoalCommand(Goal.STOW));
      elevator.setVoltage(0.25);
    }  
    
        if (currentGoal != lastGoal) {
            goalTimer.reset();
    }   
   lastGoal = currentGoal;

   switch (desiredGoal) {
        case STOW -> {
            elevator.setTargetHeight(Goal.STOW);
        }

        case L1 -> {
            currentGoal = Goal.L1;
        }

        case L2 -> {
            currentGoal = Goal.L2;
        }

        case L3 -> {
            currentGoal = Goal.L3;
        }
   }

}

    private void setGoal(Goal goal) {
        if (desiredGoal == goal) return;
        desiredGoal = goal;
        
    }

    public Command setGoalCommand(Goal goal) {
        return startEnd(() -> setGoal(goal), () -> setGoal(Goal.STOW))
            .withName("Superstructure " + goal);
  }
}
