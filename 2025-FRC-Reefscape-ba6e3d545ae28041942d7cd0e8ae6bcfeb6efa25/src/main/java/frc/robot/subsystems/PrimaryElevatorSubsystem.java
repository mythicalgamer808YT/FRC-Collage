
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.States.ElevatorStates;

public class PrimaryElevatorSubsystem extends SubsystemBase {
    private final TalonFX leftElevatorMotor;
    private final TalonFX rightElevatorMotor;
    private final PIDController elevatorPID;

    private ElevatorStates state;
    private double currentHeight; 
    private double motorOutput;
    private boolean inBounds;

    public PrimaryElevatorSubsystem() {
        leftElevatorMotor = new TalonFX(Constants.PrimaryElevatorConsts.leftElevatorMotorID);
        rightElevatorMotor = new TalonFX(Constants.PrimaryElevatorConsts.rightElevatorMotorID);  
        elevatorPID = new PIDController(Constants.PrimaryElevatorConsts.kP, Constants.PrimaryElevatorConsts.kI, Constants.PrimaryElevatorConsts.kD);
        
       // state = ElevatorStates.STARTING_POSITION;
       // setElevatorState(state);
        leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
       currentHeight = rightElevatorMotor.getPosition().getValueAsDouble() + 0.5;

        inBounds = false;
        if(currentHeight >= ElevatorStates.MAX.primaryHeight) {
            motorOutput = -0.1;
        } else if(currentHeight <= ElevatorStates.MIN.primaryHeight ) {
            motorOutput = 0.1;
        } else {
            inBounds = true;
            motorOutput = elevatorPID.calculate(currentHeight);
        }

        setElevatorSpeed(motorOutput);
        setSmartdashboard();
    }

    public void setElevatorState(ElevatorStates state) {
        this.elevatorPID.setSetpoint(state.primaryHeight);
        this.state = state;
    }

    private void setSmartdashboard() {
       // SmartDashboard.putString("Primary elevator state", state.toString());
        SmartDashboard.putNumber("Primary elevator goal position", elevatorPID.getSetpoint());
        SmartDashboard.putBoolean("Primary elevator in bounds", inBounds);
        SmartDashboard.putNumber("Primary elevator speed", motorOutput);
        SmartDashboard.putNumber("Primary elevator position ", currentHeight);
    }

    private void setElevatorSpeed(double motor) {
        leftElevatorMotor.set(motor);
        rightElevatorMotor.set(motor);
    }
}
