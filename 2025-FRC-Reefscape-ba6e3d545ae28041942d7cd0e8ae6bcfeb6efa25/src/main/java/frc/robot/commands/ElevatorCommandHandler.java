package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.States.ElevatorStates;
import frc.robot.subsystems.InnerElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

import frc.lib.util.Utilities;

public class ElevatorCommandHandler {
    private final InnerElevatorSubsystem innerElevatorSubsystem;
    private final PrimaryElevatorSubsystem primaryElevatorSubsystem;
    private final IntakeArmSubsystem intakeArmSubsystem;
    

    private ElevatorStates state;
    private SequentialCommandGroup command; 
    private double pastHeight;
    private double pastHeight2;

    public ElevatorCommandHandler(InnerElevatorSubsystem innerElevatorSubsystem, PrimaryElevatorSubsystem primaryElevatorSubsystem, IntakeArmSubsystem intakeArmSubsystem) {
        this.innerElevatorSubsystem = innerElevatorSubsystem;
        this.primaryElevatorSubsystem = primaryElevatorSubsystem;
        this.intakeArmSubsystem = intakeArmSubsystem;
        state = ElevatorStates.STARTING_POSITION;
        setElevatorStates(ElevatorStates.STARTING_POSITION);
    }

    public SequentialCommandGroup setElevators(ElevatorStates elevatorStates) {
        // if(state != null && state.innerHeight < ElevatorStates.CRITICAL_POINT.innerHeight && 
        //         state.armAngle > ElevatorStates.CRITICAL_POINT.armAngle &&
        //         elevatorStates.armAngle < ElevatorStates.CRITICAL_POINT.armAngle) {
        //     command = new SequentialCommandGroup(new CriticalPointCommand(intakeArmSubsystem, innerElevatorSubsystem, primaryElevatorSubsystem, elevatorStates));
        // } else {
       // }
       pastHeight = innerElevatorSubsystem.getPIDGoal();
       pastHeight2 = intakeArmSubsystem.getPIDGoal();

       command = new SequentialCommandGroup(new InstantCommand(() -> setElevatorStates(elevatorStates)));


       SmartDashboard.putBoolean("sigma run", false);
       System.out.println(elevatorStates.toString());
       System.out.println(state.toString());
       if(pastHeight == ElevatorStates.PRE_INTAKE.innerHeight && pastHeight2 == ElevatorStates.PRE_INTAKE.armAngle && 
            innerElevatorSubsystem.getPIDGoal() == ElevatorStates.INTAKE.innerHeight && intakeArmSubsystem.getPIDGoal() == ElevatorStates.INTAKE.armAngle) {
        SmartDashboard.putBoolean("sigma run", true);
        command = new SequentialCommandGroup(new InstantCommand (() -> primaryElevatorSubsystem.setElevatorState(elevatorStates)),
                                                new InstantCommand(() -> intakeArmSubsystem.setArm(elevatorStates.armAngle)),
                                                new WaitCommand(1),
                                                new InstantCommand(() -> innerElevatorSubsystem.setInnerElevatorState(elevatorStates.innerHeight)));
       }
        this.state = elevatorStates;
        return command;

    }

    public ElevatorStates getCurrentState() {
        return state;
    }

    private void setElevatorStates(ElevatorStates state) {
        innerElevatorSubsystem.setInnerElevatorState(state.innerHeight);
        primaryElevatorSubsystem.setElevatorState(state);
        intakeArmSubsystem.setArm(state.armAngle);
    }
}