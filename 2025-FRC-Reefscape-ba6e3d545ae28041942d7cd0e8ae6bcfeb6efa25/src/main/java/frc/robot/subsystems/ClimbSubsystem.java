package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkFlexUtil;
import frc.robot.Constants.ClimbConsts;
import frc.robot.States.ClimbStates;


public class ClimbSubsystem extends SubsystemBase{
    private final SparkFlex climbMotor;

    public ClimbSubsystem() {
        climbMotor = new SparkFlex(ClimbConsts.climbMotorID, MotorType.kBrushless);
        SparkFlexUtil.setSparkFlexBusUsage(climbMotor, SparkFlexUtil.Usage.kAll, IdleMode.kBrake, false, true);
    }

    public void setClimbState(ClimbStates climbState) {
        climbMotor.set(climbState.speed);
    }

}
