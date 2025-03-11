
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
import frc.robot.Constants.IntakeArmConsts;
import frc.robot.States.ElevatorStates;
import frc.robot.States.IndexStates;

public class IntakeArmSubsystem extends SubsystemBase {   
    private final TalonFX armMotor;
    private final SparkMax indexingMotor;
    private final DutyCycleEncoder armEncoder; 
    private final ArmFeedforward armFeedforward;
    private final PIDController armPID;

    private double armPosition;
    private double motorSpeed; 
    public boolean inBounds; 

    public IntakeArmSubsystem() {
        armMotor = new TalonFX(Constants.IntakeArmConsts.armMotorID);
        indexingMotor = new SparkMax(Constants.IntakeArmConsts.suckMotorID, MotorType.kBrushless);
        armEncoder = new DutyCycleEncoder(Constants.IntakeArmConsts.encoderID);

        armFeedforward = new ArmFeedforward(Constants.IntakeArmConsts.kS, Constants.IntakeArmConsts.kG, Constants.IntakeArmConsts.kV);
        armPID = new PIDController(Constants.IntakeArmConsts.kP, Constants.IntakeArmConsts.kI, Constants.IntakeArmConsts.kD);

        armPID.setTolerance(IntakeArmConsts.PID_TOLERANCE);
    }

    @Override
    public void periodic() {
        armPosition = getAngle();
        inBounds = true;

        if(armPosition >= ElevatorStates.MAX.armAngle) {
            // posotive is up
            inBounds = false;
            motorSpeed = -0.1;
        } else if(armPosition <= ElevatorStates.MIN.armAngle) {
            inBounds = false;
            motorSpeed = 0.1;
        } else {
            motorSpeed = armPID.calculate(armPosition) + armFeedforward.calculate(Units.degreesToRadians(armPosition), armMotor.getVelocity().getValueAsDouble());
        }

        armMotor.set(motorSpeed);
        setSmartdashboard();
    }

    public boolean atAngle() {
        return armPID.atSetpoint();
    }

    public void setAngle(double angle) {
        armPID.setSetpoint(angle);
    }

    public void changeAngle(double angleDelta) {
        armPID.setSetpoint(getPIDGoal() + angleDelta);
    }

    public void setIndex(double speed) {
        indexingMotor.set(speed);
    }

    public double getAngle() {
        return Units.rotationsToDegrees(armEncoder.get() - Units.degreesToRotations(87));
    }
    
    public double getPIDGoal() {
        return armPID.getSetpoint();
    }
    
    private void setSmartdashboard() {
        SmartDashboard.putNumber("Arm Angle", getAngle());
        SmartDashboard.putNumber("Arm Motor Speed", motorSpeed);
        SmartDashboard.putNumber("Arm Angle Goal", armPID.getSetpoint());
    }
}
