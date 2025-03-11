
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.States.ElevatorStates;
import frc.robot.States.IndexStates;

public class IntakeArmSubsystem extends SubsystemBase {   
    private final TalonFX armMotor;
    private final SparkMax indexingMotor;
    private final DutyCycleEncoder armEncoder; 
    private final ArmFeedforward armFeedforward;
    private final PIDController armPID;

    private IndexStates indexState;
    private ElevatorStates state;
    private double armPosition;
    private double motorSpeed; 
    public boolean override; 

    public IntakeArmSubsystem() {
        armMotor = new TalonFX(Constants.IntakeArmConsts.armMotorID);
        indexingMotor = new SparkMax(Constants.IntakeArmConsts.suckMotorID, MotorType.kBrushless);
        armEncoder = new DutyCycleEncoder(Constants.IntakeArmConsts.encoderID);

        armFeedforward = new ArmFeedforward(Constants.IntakeArmConsts.kS, Constants.IntakeArmConsts.kG, Constants.IntakeArmConsts.kV);
        armPID = new PIDController(Constants.IntakeArmConsts.kP, Constants.IntakeArmConsts.kI, Constants.IntakeArmConsts.kD);

       // state = ElevatorStates.STARTING_POSITION;
        indexState = IndexStates.STOP;
        setIndexState(indexState);
       // setArm(state.armAngle);
    }

    @Override
    public void periodic() {
        armPosition = getAngle();
        
        if(armPosition >= ElevatorStates.MAX.armAngle) {
            // posotive is up
            motorSpeed = -0.1;
        } else if(armPosition <= ElevatorStates.MIN.armAngle) {
            motorSpeed = 0.1;
        } else if(!override) {
            motorSpeed = armPID.calculate(armPosition) + armFeedforward.calculate(Units.degreesToRadians(armPosition), armMotor.getVelocity().getValueAsDouble());
        }

        armMotor.set(motorSpeed);
        setSmartdashboard();
    }

    public void setArm(double angle) {
        //this.state = state;
        armPID.setSetpoint(angle);
    }

    public void setIndexState(IndexStates state) {
        indexingMotor.set(state.speed);
        this.indexState = state;
    }

    public void setOverride(double motorSpeed, boolean override) {
        this.override = override;
        this.motorSpeed = motorSpeed;
    }

    public double getAngle() {
        return Units.rotationsToDegrees(armEncoder.get() - Units.degreesToRotations(87));
    }
    
    public double getPIDGoal() {
        return armPID.getSetpoint();
    }
    
    private void setSmartdashboard() {
        SmartDashboard.putString("Arm Subsytem index state", indexState.toString());
      //  SmartDashboard.putString("Arm Subsystem arm state ", state.toString());
        SmartDashboard.putNumber("Arm Subsystem position", getAngle());
        SmartDashboard.putNumber("Arm Subsystem motor speed", motorSpeed);
        SmartDashboard.putNumber("Arm Subsystem arm position goal", armPID.getSetpoint());
    }
}
