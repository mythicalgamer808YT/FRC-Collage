package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.States.ClimbStates;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand {
    private final ClimbSubsystem climb; 
    private ClimbStates climbState;

    public ClimbCommand(ClimbSubsystem climb) {
        this.climb = climb;
        climbState = ClimbStates.STOP;
        climb.setClimbState(climbState);
    } 

    public ClimbStates getCurrentState() {
        return climbState;
    }

    public Command setClimb(ClimbStates state) {
        climbState = state;
        return new InstantCommand(() -> climb.setClimbState(state));
    }
}
