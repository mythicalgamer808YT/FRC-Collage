package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PhotonConsts;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonSubsystem2;

public class PhotonCommand2 extends Command{
    private final PhotonSubsystem2 photon;
    private CommandSwerveDrivetrain drivetrain;
    private SwerveRequest.RobotCentric robotCentricDrive;
    
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rController;

    private boolean noTargetFound;
    private boolean withinDesiredRange;
    private final boolean moveLeft;

    private double lastTime;
    private double currentTime;

    public PhotonCommand2(PhotonSubsystem2 photonSubsystem, CommandSwerveDrivetrain drivetrain, boolean moveLeft) {
        this.photon = photonSubsystem;
        this.drivetrain = drivetrain;
        this.robotCentricDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        this.withinDesiredRange = false;
        this.noTargetFound = false;
        this.moveLeft = true;

        lastTime = Timer.getFPGATimestamp();

        xController = new PIDController(2, 0, 0);
        yController = new PIDController(2, 0, 0);
        rController = new PIDController(0.1, 0, 0);

        rController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        Optional<Translation2d> targetTranslationOpt = photon.getClosestTargetTranslation();
        currentTime = Timer.getFPGATimestamp();

        if(targetTranslationOpt.isEmpty()) {
            noTargetFound = true;
            return;
        }
        noTargetFound = false;

        Translation2d targetTranslation = targetTranslationOpt.get();
        
        Translation2d desiredTranslation = moveLeft ? PhotonConsts.LEFT_DESIRED_TRANSLATION
                                                    : PhotonConsts.RIGHT_DESIRED_TRANSLATION;

        double desiredAngle = Math.atan2(targetTranslation.getY() - desiredTranslation.getY(),
                                        targetTranslation.getX() - desiredTranslation.getX());

        SmartDashboard.putNumber("Desired Translation X", desiredTranslation.getX());
        SmartDashboard.putNumber("Desired Translation Y", desiredTranslation.getY());
        SmartDashboard.putNumber("Computed Desired Angle", desiredAngle);

        if(desiredTranslation.getNorm() < PhotonConsts.DESIRED_RANGE) {
            withinDesiredRange = true;
            return;
        }

        double xSpeed = xController.calculate(0, desiredTranslation.getX());
        double ySpeed = yController.calculate(0, desiredTranslation.getY());
        double rSpeed = rController.calculate(0, desiredAngle);//desiredTranslation.getAngle().getRadians());

        drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(rSpeed)).execute();

        lastTime = Timer.getFPGATimestamp();

    }

    @Override
    public void end(boolean isFinished) {
        drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)).execute();
    }

    @Override
    public boolean isFinished() {
        return (noTargetFound && currentTime - lastTime > PhotonConsts.TIMEOUT_COMMAND) || withinDesiredRange;
    }
}
