// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.States.ElevatorStates;
// import frc.robot.subsystems.InnerElevatorSubsystem;
// import frc.robot.subsystems.IntakeArmSubsystem;
// import frc.robot.subsystems.PrimaryElevatorSubsystem;

// public class CriticalPointCommand extends Command {
//     private final IntakeArmSubsystem intakeArm;
//     private final InnerElevatorSubsystem elevator;
//     private final PrimaryElevatorSubsystem primaryElevator;
//     private final ElevatorStates desiredElevatorStates;
    

//     private boolean reachedIntakeArm;

//     public CriticalPointCommand(IntakeArmSubsystem i, InnerElevatorSubsystem e, PrimaryElevatorSubsystem ee, ElevatorStates desiredStates) {
//         this.intakeArm = i; 
//         this.elevator = e; 
//         this.desiredElevatorStates = desiredStates;
//         this.primaryElevator = ee;
//         this.reachedIntakeArm = false;
//     }

//     @Override
//     public void initialize()  {
//         primaryElevator.setElevatorState(desiredElevatorStates);
//         //elevator.setInnerElevatorState(ElevatorStates.PRE_INTAKE);
//     }

//     @Override
//     public void execute() {
        
//         if(desiredElevatorStates.innerHeight < ElevatorStates.CRITICAL_POINT.innerHeight && elevator.getHeight() >= ElevatorStates.CRITICAL_POINT.innerHeight) {
//             double newAngle = Math.max(ElevatorStates.CRITICAL_POINT.armAngle, desiredElevatorStates.armAngle);
//             intakeArm.setArm(newAngle);
//             if(Math.abs(intakeArm.getAngle() - newAngle) < 0.05) {
//                 elevator.setInnerElevatorState(desiredElevatorStates.innerHeight);
//             }
//         } else if(desiredElevatorStates.innerHeight < ElevatorStates.CRITICAL_POINT.innerHeight) {// && desiredElevatorStates.armAngle < ElevatorStates.CRITICAL_POINT.armAngle) {
//             elevator.setInnerElevatorState(ElevatorStates.CRITICAL_POINT.innerHeight + 2);
//             intakeArm.setArm(Math.min(ElevatorStates.CRITICAL_POINT.armAngle + 2, intakeArm.getAngle()));
//         } else {
//             intakeArm.setArm(desiredElevatorStates.armAngle);
//             elevator.setInnerElevatorState(desiredElevatorStates.innerHeight);
//         }

//         // if(Math.abs(elevator.distanceFromSetpoint()) < 0.05) {
//         //     intakeArm.setArmState(desiredElevatorStates);
//         // }
//     }

//     @Override
//     public void end(boolean isFinished) {
//         elevator.setInnerElevatorState(desiredElevatorStates.innerHeight);
//     }

//     @Override
//     public boolean isFinished() {
//         if(Math.abs(elevator.distanceFromSetpoint()) < 0.02 && Math.abs(intakeArm.getAngle() - desiredElevatorStates.armAngle) < 0.05) {
//             return true;
//         }
//         return false;
//     }
    
// }
