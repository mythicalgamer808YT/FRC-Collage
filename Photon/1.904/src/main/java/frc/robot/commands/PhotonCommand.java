// package frc.robot.commands;

// import java.util.NoSuchElementException;
// import java.util.Optional;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.PhotonConsts;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.PhotonSubsystem;

// public class PhotonCommand extends Command {
//     private final PhotonSubsystem photon;
//     private final CommandSwerveDrivetrain drivetrain;
//     private final SwerveRequest.RobotCentric robotCentricDrive; 

//     //calculate the PID values
//     private final ProfiledPIDController xController;
//     private final ProfiledPIDController yController;
//     private final ProfiledPIDController rController;

//     private final Translation2d desirTranslation2d;

//     private int targetAprilTagID;
//     private boolean availablePose = false;
//     private boolean noTargetFound;
//     private boolean withinDesiredRange;
//     private boolean cancelCommand; 
    

//     private Pose2d aprilTagPose;
    
//     private double xSpeed;
//     private double ySpeed;
//     private double rSpeed;

//     private double currentTime;
//     private double lastTime;
    

//     //implement swerve drive subsystem

//     public PhotonCommand(PhotonSubsystem photonSubsystem, CommandSwerveDrivetrain drivetrain, Translation2d desiredTranslation2d) {
//         this.photon = photonSubsystem;
//         this.drivetrain = drivetrain;
//         this.robotCentricDrive = new RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

//         this.desiredTranslation2d = desiredTranslation2d;

//         this.noTargetFound = false;
//         this.withinDesiredRange = false;

//         lastTime = Timer.getFPGATimestamp();

//         xController = new ProfiledPIDController(0.5, 0, 0, PhotonConsts.translationConstraints);
//         yController = new ProfiledPIDController(0.5, 0, 0, PhotonConsts.translationConstraints);
//         rController = new ProfiledPIDController(2, 0, 0, PhotonConsts.rotationConstraints);

//         rController.enableContinuousInput(-Math.PI, Math.PI);
//     }

//     @Override
//     public void initialize() {
        
            
//     }

//     @Override
//     public void execute() {
//         try {
//             //get el target
//             targetAprilTagID = photon.getClosestTarget().get()[0].intValue();

//         } catch(NoSuchElementException error) {
//             noTargetFound = true;
//         }
//         Pose3d estimatedPose3d = photon.getCollectiveEstimatedPose();

//         if(estimatedPose3d == null) {
//             noTargetFound = true;
//             return; 
//         }

//         Pose2d estimatedPose2d = estimatedPose3d.toPose2d();
//         //double targetDistance = estimatedPose2d.getTranslation().getNorm();

//         // SmartDashboard.putNumber("Aptil Tag Pose Opt X", aprilTagPose.getX());
//         // SmartDashboard.putNumber("Aptil Tag Pose Opt Y", aprilTagPose.getY());
//         // SmartDashboard.putNumber("Aptil Tag Pose Opt Rotation", aprilTagPose.getRotation().getRadians());


//         double offsetDistance = 1.0; // meters
//         Translation2d tagTranslation = estimatedPose2d.getTranslation();
//         Rotation2d tagRotation = estimatedPose2d.getRotation();

//         SmartDashboard.putNumber("Tag Translation X", tagTranslation.getX());
//         SmartDashboard.putNumber("Tag Translation Y", tagTranslation.getY());
//         SmartDashboard.putNumber("Tag Rotation", tagRotation.getRadians());

//         Translation2d desiredTranslation = tagTranslation.plus(
//             new Translation2d(-offsetDistance, new Rotation2d(tagRotation.getRadians()))
//         );

//         SmartDashboard.putNumber("Desired Translation X", desiredTranslation.getX());
//         SmartDashboard.putNumber("Desired Translation Y", desiredTranslation.getY());
//         SmartDashboard.putNumber("Desired Rotation", desiredTranslation.getAngle().getRadians());

//         //Pose2d targetPose = new Pose2d(desiredTranslation, tagRotation);

//         //calculate difference from target
//         if(desiredTranslation.getNorm() < PhotonConsts.DESIRED_RANGE && desiredTranslation.getAngle().getRadians() < PhotonConsts.DESIRED_RANGE) {
//             withinDesiredRange = true;
//             return;
//         }

//         xSpeed = xController.calculate(estimatedPose2d.getX(), desiredTranslation.getX());
//         ySpeed = yController.calculate(estimatedPose2d.getY(), desiredTranslation.getY());
//         rSpeed = rController.calculate(estimatedPose2d.getRotation().getRadians(), desiredTranslation.getAngle().getRadians());

//         // drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(xSpeed)
//         //     .withVelocityY(ySpeed)
//         //     .withRotationalRate(rSpeed)).execute();

//         currentTime = Timer.getFPGATimestamp();
//     }

//     @Override
//     public void end(boolean isFinished) {
//         // drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(0)
//         //     .withVelocityY(0)
//         //     .withRotationalRate(0)).execute();
//     }

//     @Override
//     public boolean isFinished() {
//         if(noTargetFound && currentTime - lastTime > PhotonConsts.TIMEOUT_COMMAND) {
//             return true;
//         }
        
//         if(withinDesiredRange) {
//             return true;
//         }
        
//         return false;
//     }
// }