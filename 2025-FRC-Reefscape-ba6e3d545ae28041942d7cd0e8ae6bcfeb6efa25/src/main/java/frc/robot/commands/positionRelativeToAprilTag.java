package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.time.zone.ZoneRulesException;
import java.util.EmptyStackException;
import java.util.NoSuchElementException;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.PhotonStates;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonSubsystem1;

public class positionRelativeToAprilTag extends Command{
    private PhotonSubsystem1 camera;
    private SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private CommandSwerveDrivetrain drivetrain;
    private double goalYaw = -10;
    private double goalX= 0.43;
    private double goalY = 0.2; 
    private PIDController xController;
    private PIDController yController;
    private PIDController theataController;


    public positionRelativeToAprilTag(PhotonSubsystem1 camera, CommandSwerveDrivetrain drive) {
        this.camera = camera;
        this.drivetrain = drive;
        xController = new PIDController(2, 0, 0);
        yController = new PIDController(2, 0, 0);
        theataController = new PIDController(0.1, 0,0 );
        theataController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(camera);
    }

    @Override
    public void initialize() { 
        xController.setSetpoint(goalX);
        yController.setSetpoint(goalY);
        theataController.setSetpoint(goalYaw);

        xController.setTolerance(0.01);
        yController.setTolerance(0.01);
        theataController.setTolerance(0.01);
        camera.resetYaw();
    }

    @Override
    public void execute() {
            Transform3d sigma = camera.getDistances();

            if(sigma == null) {
                return;
            }
        // if(camera.getYaw()) {
        //     double yaw = camera.getYaw().get();
        double yaw;

        try {
            yaw = camera.getYaw().get();
        } catch (NoSuchElementException e) {
            yaw = 0;
        }
        
        double speed = -xController.calculate(sigma.getX());
        double speed1 = -yController.calculate(sigma.getY());
        double yawSpeed = -theataController.calculate(Units.degreesToRadians(yaw));

        if(!camera.canRotate()) {
            yawSpeed = 0;
        }

        double realYawSpeed = yawSpeed;
        drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(speed)
        .withVelocityY(speed1).withRotationalRate(realYawSpeed)).execute();

        SmartDashboard.putNumber("snhaoetunshao", yawSpeed);
        SmartDashboard.putNumber("x speed", speed);
        SmartDashboard.putNumber("sigma speed", speed1);
        SmartDashboard.putNumber("goal ", sigma.getX());
        SmartDashboard.putNumber("nthaoeu", sigma.getY());
    }

    @Override
    public void end(boolean inFinished) {
        drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(0)
        .withVelocityY(0).withRotationalRate(0)).execute();
    }

    @Override
    public boolean isFinished() {
        if(xController.atSetpoint() && yController.atSetpoint() && theataController.atSetpoint()) {
            return true;
        }
        return false;
    }
}