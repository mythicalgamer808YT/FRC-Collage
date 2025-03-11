// package frc.robot.subsystems;

// import java.util.List;
// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.numbers.*;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.PhotonConsts;
// import frc.robot.commands.PhotonCommand;

// //built for multiple cameras, finds average pose of every reading then 
// public class PhotonSubsystem extends SubsystemBase {
//     private CommandSwerveDrivetrain drivetrain;

//     private Matrix<N3, N1> curStdDevs = PhotonConsts.SINGLE_STD_DEVS; // dynamic standard deviation

//     private List<PhotonCamera> cameras; // pain transforms and cameras later
//     private List<Transform3d> cameraToRobotTransforms;
//     private Optional<Pose3d> cameraPose = Optional.of(new Pose3d());
//     private int targetID;

//     private double lastUpdateTime = 0;
//     private Optional<Pose3d> lastEstimatedPose = Optional.empty();
//     private Optional<Double[]> closestValidTarget = Optional.empty();

//     private boolean targetExist = false;//testing
//     private boolean poseExist = false;
    
//     public PhotonSubsystem(List<String> cameraNames, List<Transform3d> cameraToRobotTransforms, CommandSwerveDrivetrain drivetrain) {

//         this.cameraToRobotTransforms = cameraToRobotTransforms;//was throwin error
//         this.cameras = cameraNames.stream().map(PhotonCamera::new).toList();

//         this.drivetrain = drivetrain;

//         //this.closestValidTarget = Optional.empty();
//     }

//     //for swerve drive estimator type stuff
//     private void updateEstimationStdDevs(int numTags, double avgDist) {
//             //if 1 and far then standard deviation is larger, else smaller
//         curStdDevs = numTags > 1 ? VecBuilder.fill(0.5, 0.5, 1) : VecBuilder.fill(4, 4, 8);
//         if(numTags == 1 && avgDist > 4) {//dist in meters
//             curStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
//         } else {
//             curStdDevs = curStdDevs.times(1 + (avgDist * avgDist / 30));
//         }
//     }

//     public Matrix<N3, N1> getEstimationStdDevs() {
//         return curStdDevs;
//     }

//     @Override
//     public void periodic() {
//         Pose3d addedPose = new Pose3d(); // find average pose, couldn't figure a List or array list for some reason
//         double avgDist = 0;//for standard deviation
//         int validPoseCount = 0; //for average pose among cameras, could also just get best target overall and best camera
//         int index = 0;//which cam for pose transforms
//         //collectiveEstimatedPose = null;

//         double currentTime = Timer.getFPGATimestamp();

//         targetExist = false;
//         for(PhotonCamera cam : cameras) {
//             List<PhotonPipelineResult> results = cam.getAllUnreadResults();
           
//             for(PhotonPipelineResult result : results) {

//                 List<PhotonTrackedTarget> filteredTargets = result.getTargets().stream()
//                             .filter(target -> target.getPoseAmbiguity() < PhotonConsts.MIN_AMBIGUITY)
//                             .toList();

//                 for(PhotonTrackedTarget target : filteredTargets) {
//                     targetExist = true;

//                     Pose3d tempCameraPose = new Pose3d().transformBy(cameraToRobotTransforms.get(index));
//                     int currentTarget = target.getFiducialId();

//                     double dist = tempCameraPose.toPose2d().getTranslation().getNorm();
//                     avgDist += dist;
//                     setClosestTarget(dist, currentTarget);

//                     if(currentTarget == targetID) {
//                         addedPose = new Pose3d(
//                             addedPose.getX() + tempCameraPose.getX(),
//                             addedPose.getY() + tempCameraPose.getY(),
//                             addedPose.getZ() + tempCameraPose.getZ(),
//                             new Rotation3d(
//                                 addedPose.getRotation().getX() + tempCameraPose.getRotation().getX(),
//                                 addedPose.getRotation().getY() + tempCameraPose.getRotation().getY(),
//                                 addedPose.getRotation().getZ() + tempCameraPose.getRotation().getZ()
//                             )
//                         );
//                         validPoseCount++;
//                     }
                    
//                 }
//                 index++;
//             }

//             if(validPoseCount > 0) {
//                 updateEstimationStdDevs(index, avgDist);
//                 lastUpdateTime = currentTime; 

//                 cameraPose = Optional.of(new Pose3d(
//                     addedPose.getX() / validPoseCount,
//                     addedPose.getY() / validPoseCount,
//                     addedPose.getZ() / validPoseCount,
//                     new Rotation3d(
//                         addedPose.getRotation().getX() / validPoseCount,
//                         addedPose.getRotation().getY() / validPoseCount,
//                         addedPose.getRotation().getZ() / validPoseCount
//                     )
//                 ));

//             }
//         }
           
           
//             // for(PhotonPipelineResult result : results) {
//         //         //check if targets are resonable
//         //         List<PhotonTrackedTarget> filteredTargets = result.getTargets().stream()
//         //             .filter(target -> target.getPoseAmbiguity() < PhotonConsts.MIN_AMBIGUITY)
//         //             .toList();

//         //         SmartDashboard.putNumber("amt Targets", filteredTargets.size());

//         //         for(PhotonTrackedTarget target : filteredTargets) {
//         //             targetExist = true;
//         //             //Optional<Pose3d> tagPoseOpt = PhotonConsts.aprilTagFieldLayout.getTagPose(target.getFiducialId());
//         //             //check if pose was resonable
//         //             //if(tagPoseOpt.isPresent()) {
//         //             poseExist = true;
//         //             //transforms here to utalize index, consider creating a higher level pose handler
//         //             //Pose3d tagPose = tagPoseOpt.get();
//         //             // SmartDashboard.putNumber("TAG POSE X", tagPose.getX());
//         //             // SmartDashboard.putNumber("TAG POSE Y", tagPose.getY());
//         //             Pose3d cameraPose = new Pose3d().transformBy(target.getBestCameraToTarget().inverse());
//         //             //Pose3d cameraPose = tagPose.transformBy(target.getBestCameraToTarget());
//         //             //Pose3d transformedPose = cameraPose.transformBy(cameraToRobotTransforms.get(index));

//         //             double targetDistance = cameraPose.toPose2d().getTranslation().getNorm();
//         //             setClosestTarget(targetDistance, target.getFiducialId());

//         //             avgDist += targetDistance;
//         //             addedPose = new Pose3d(
//         //                 addedPose.getX() + cameraPose.getX(),
//         //                 addedPose.getY() + cameraPose.getY(),
//         //                 addedPose.getZ() + cameraPose.getZ(),
//         //                 new Rotation3d(
//         //                     addedPose.getRotation().getX() + cameraPose.getRotation().getX(),
//         //                     addedPose.getRotation().getY() + cameraPose.getRotation().getY(),
//         //                     addedPose.getRotation().getZ() + cameraPose.getRotation().getZ()
//         //                 )
//         //             );

//         //             validPoseCount++;
//         //             //}
//         //         }

//         //     }
//         //     index++;
//         // }

//         // if(validPoseCount > 0) {
            
//         //     updateEstimationStdDevs(validPoseCount, avgDist / validPoseCount);
//         //     lastUpdateTime = currentTime;

//         //     collectiveEstimatedPose = new Pose3d(
//         //                     addedPose.getX() / validPoseCount,
//         //                     addedPose.getY() / validPoseCount,
//         //                     addedPose.getZ() / validPoseCount,
//         //                     new Rotation3d(
//         //                         addedPose.getRotation().getX() / validPoseCount,
//         //                         addedPose.getRotation().getY() / validPoseCount,
//         //                         addedPose.getRotation().getZ() / validPoseCount
//         //                     )
//         //                 );
//         //     if(lastEstimatedPose.isPresent()) {
//         //         Pose3d lastPose = lastEstimatedPose.get();

//         //         if(Math.abs(lastPose.getTranslation().getNorm() - collectiveEstimatedPose.getTranslation().getNorm()) > 0.3 
//         //             || Math.abs(lastPose.getRotation().getZ() - collectiveEstimatedPose.getRotation().getZ()) > 0.5) {
//         //                 Rotation3d collectiveRotation = collectiveEstimatedPose.getRotation();
//         //                 Rotation3d lastRotation = lastPose.getRotation();

//         //                 collectiveEstimatedPose = new Pose3d(
//         //                     collectiveEstimatedPose.getX() + 0.01 * (collectiveEstimatedPose.getX() - lastPose.getX()),
//         //                     collectiveEstimatedPose.getY() + 0.01 * (collectiveEstimatedPose.getY() - lastPose.getY()),
//         //                     collectiveEstimatedPose.getZ() + 0.01 * (collectiveEstimatedPose.getZ() - lastPose.getZ()),
//         //                     new Rotation3d(
//         //                         collectiveRotation.getX() + 0.01 * (collectiveRotation.getX() - lastRotation.getX()),
//         //                         collectiveRotation.getY() + 0.01 * (collectiveRotation.getY() - lastRotation.getY()),
//         //                         collectiveRotation.getZ() + 0.01 * (collectiveRotation.getZ() - lastRotation.getZ())
//         //                     )
//         //                 );
//         //         }

//         //         lastEstimatedPose = Optional.of(collectiveEstimatedPose);
//         //     }

//         //     SmartDashboard.putNumber("valid Poses", validPoseCount);
        
//         // } else if (currentTime - lastUpdateTime > PhotonConsts.TIMEOUT_SUBSYSTEM) {
//         //     collectiveEstimatedPose = null; //clear result if too great of time difference
//         //     closestValidTarget = Optional.empty();
//         //     lastEstimatedPose = Optional.empty();
//         //     //no change to swerve pose
//         // }

//         // SmartDashboard.putNumber("Distance", avgDist);
//         // setSmartDashboard();
//     }

//     public void setTarget(int targetID) {
//         this.targetID = targetID;
//     }

//     //Double[targetDistance, targetID]
//     public void setClosestTarget(double targetDistance, double targetID) {
//         if(closestValidTarget.isPresent()) {
//             if(closestValidTarget.get()[0].doubleValue() < targetDistance) {
//                 closestValidTarget = Optional.of(new Double[]{Double.valueOf(targetDistance), Double.valueOf(targetID)});
//             }
//         } else {
//             closestValidTarget = Optional.of(new Double[]{Double.valueOf(targetDistance), Double.valueOf(targetID)});
//         }
//     }

//     public Optional<Double[]> getClosestTarget() {
//         return closestValidTarget;
//     }

//     public Optional<Pose3d> getCameraPose() {
//         return cameraPose;
//     }

//     //distance and angle to specific tag
//     public void setSmartDashboard() {
//         Pose3d daPose = cameraPose.get();
//         if(poseExist) {
//             SmartDashboard.putNumber("Pose X", daPose.getX());
//             SmartDashboard.putNumber("Pose Y", daPose.getY());
//             SmartDashboard.putNumber("Pose theata", daPose.getRotation().getZ());            
//         } else {
//             SmartDashboard.putString("Robot Pose", "No pose available");
//         }
//         SmartDashboard.putBoolean("There is Target", targetExist);
//     }
// }
