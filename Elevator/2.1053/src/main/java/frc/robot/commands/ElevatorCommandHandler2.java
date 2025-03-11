package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.ElevatorStates;
import frc.robot.subsystems.InnerElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

public class ElevatorCommandHandler2 {
    private final InnerElevatorSubsystem innerElevatorSubsystem;
    private final PrimaryElevatorSubsystem primaryElevatorSubsystem;
    private final IntakeArmSubsystem intakeArmSubsystem;

    private ElevatorStates state;
    private Command command;

    public ElevatorCommandHandler2(InnerElevatorSubsystem innerElevatorSubsystem, PrimaryElevatorSubsystem primaryElevatorSubsystem, IntakeArmSubsystem intakeArmSubsystem) {
        this.innerElevatorSubsystem = innerElevatorSubsystem;
        this.primaryElevatorSubsystem = primaryElevatorSubsystem;
        this.intakeArmSubsystem = intakeArmSubsystem;
        
    }

    public Command setElevatorState(ElevatorStates elevatorStates) {
        state = elevatorStates;

        innerElevatorSubsystem.setInnerElevatorHeight(elevatorStates.innerHeight);
        primaryElevatorSubsystem.setPrimaryElevatorHeight(elevatorStates.primaryHeight);
        intakeArmSubsystem.setAngle(elevatorStates.armAngle);

        return command;
    }

    public ElevatorStates getCurrentState() {
        return state;
    }
}
