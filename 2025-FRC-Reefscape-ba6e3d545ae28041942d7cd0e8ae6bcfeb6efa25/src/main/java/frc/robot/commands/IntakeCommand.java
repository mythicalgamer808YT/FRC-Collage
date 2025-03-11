package frc.robot.commands;

import frc.robot.States.IndexStates;
import frc.robot.subsystems.IntakeArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeCommand {
    private final IntakeArmSubsystem intakeArm;

    public IntakeCommand (IntakeArmSubsystem ias) {
        this.intakeArm = ias;
    }

    public Command setIntakeState(IndexStates state) {
        return new InstantCommand(() -> intakeArm.setIndexState(state));
    }
}
