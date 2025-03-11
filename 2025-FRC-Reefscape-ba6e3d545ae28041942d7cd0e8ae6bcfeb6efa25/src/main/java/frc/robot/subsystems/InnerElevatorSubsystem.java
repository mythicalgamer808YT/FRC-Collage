package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.SparkFlexUtil;
import frc.lib.util.Utilities;
import frc.lib.util.SparkFlexUtil.Usage;
import frc.robot.Constants;
import frc.robot.States.ElevatorStates;

public class InnerElevatorSubsystem extends SubsystemBase {
    private final SparkFlex elevatorMotor;
    private final PIDController elevatorPID;
   // private final ArmFeedforward armFeedforward;
    //private final SparkAbsoluteEncoder elevatorEncoder;
    private final RelativeEncoder elevatorEncoder;
    private ElevatorStates state;
    private double currentPosition; 
    private double motorSpeed;
    private boolean inBounds;

    public InnerElevatorSubsystem() {
        elevatorMotor = new SparkFlex(Constants.InnerElevatorConsts.elevatorMotorID, MotorType.kBrushless);
        SparkFlexUtil.setSparkFlexBusUsage(elevatorMotor, SparkFlexUtil.Usage.kAll, IdleMode.kBrake, false, false);
        //elevatorEncoder = elevatorMotor.getAbsoluteEncoder();
        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorPID = new PIDController(Constants.InnerElevatorConsts.kP, Constants.InnerElevatorConsts.kI, Constants.InnerElevatorConsts.kD);

       // state = ElevatorStates.STARTING_POSITION;
        currentPosition = 0; 
    }

    @Override
    public void periodic() {
        currentPosition = getHeight(); 
        inBounds = false;

        if(currentPosition >= ElevatorStates.MAX.innerHeight) {
            // positive goes up 
            motorSpeed = -0.1;
        } else if (currentPosition <= ElevatorStates.MIN.innerHeight) {
            motorSpeed = 0.1;
        } else {
            motorSpeed = elevatorPID.calculate(currentPosition) + 0.05;
            if(motorSpeed < -0.1) {
                motorSpeed *= 0.2;
            }
        }

        elevatorMotor.set(motorSpeed);
        setSmartdashboard();
    }

    public void setInnerElevatorState(double height) {
        this.elevatorPID.setSetpoint(height);
        //this.state = state;
    }

    public double distanceFromSetpoint() {
        if(state == null) {
            return 0.0;
        }
        return state.innerHeight - this.currentPosition;
    }

    public double getPIDGoal() {
        return elevatorPID.getSetpoint();
    }

    public double getHeight() {
        return elevatorEncoder.getPosition() + 20;
    }

    private void setSmartdashboard() {
      //  SmartDashboard.putString("Inner elevator state", state.toString());
        SmartDashboard.putBoolean("Inner elevator in bounds", inBounds);
        SmartDashboard.putNumber("Inner elevator speed", motorSpeed);
        SmartDashboard.putNumber("Inner elevator posotion ", currentPosition);
        SmartDashboard.putNumber("Inner elevator goal position", elevatorPID.getSetpoint());
    }
}