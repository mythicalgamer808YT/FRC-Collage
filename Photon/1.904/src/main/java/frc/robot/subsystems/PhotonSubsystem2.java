package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonConsts;

public class PhotonSubsystem2 extends SubsystemBase{
    private Matrix<N3,N1> curStdDevs = PhotonConsts.SINGLE_STD_DEVS;

    private List<PhotonCamera> cameras;
    private List<Transform3d> cameraToRobotTransforms;

    private List<Integer> reefIDs;//switch to Set if theres a lotta reef ID's
    private Translation2d closestTargetTranslation = null;
    private int currentTargetID = -1;
    private double lastSeenTime = 0;

    //nga see
    private boolean targetFound = false;

    public PhotonSubsystem2() {
        this.cameraToRobotTransforms = PhotonConsts.CAM_TO_ROBOT_TRANSFORMS;
        this.cameras = PhotonConsts.CAM_NAMES.stream().map(PhotonCamera::new).toList();

        this.reefIDs = PhotonConsts.VALID_REEF_IDS;
    }

    @Override
    public void periodic() {
        Optional<Translation2d> targetTranslationOpt = getRobotToTargetTranslation();

        if(targetTranslationOpt.isPresent()) {
            Translation2d newTargetTranslation = targetTranslationOpt.get();
            int newTargetID = getClosestValidTargetID(); // when closest target switches

            if(newTargetID == currentTargetID) {
                lastSeenTime = Timer.getFPGATimestamp();
            } else if(Timer.getFPGATimestamp() - lastSeenTime > PhotonConsts.TARGET_LOST_TIMEOUT) {
                currentTargetID = newTargetID;
                closestTargetTranslation = newTargetTranslation;
                lastSeenTime = Timer.getFPGATimestamp();
            }

            targetFound = true;
        } else { // don't see crap
            if(Timer.getFPGATimestamp() - lastSeenTime > PhotonConsts.TARGET_LOST_TIMEOUT) {
                closestTargetTranslation = null;
                targetFound = false;
                currentTargetID = -1;
            }
        }

        SmartDashboard.putBoolean("Target Found", targetFound);
        SmartDashboard.putNumber("Current Target ID", currentTargetID);
        if(targetFound) {
            SmartDashboard.putNumber("Closest Target X", closestTargetTranslation.getX());
            SmartDashboard.putNumber("Closest Target Y", closestTargetTranslation.getY());
        }
    }

    public Optional<Translation2d> getRobotToTargetTranslation() {
        double minDistance = Double.MAX_VALUE;
        Translation2d closestTranslation = null;
        int closestID = -1;

        for(PhotonCamera cam : cameras) {
            List<PhotonPipelineResult> results = cam.getAllUnreadResults();

            for(PhotonPipelineResult result : results) {
                List<PhotonTrackedTarget> validTargets = result.getTargets().stream()
                    .filter(target -> reefIDs.contains(target.getFiducialId()))//only allow reef Targets
                    .filter(target -> target.getPoseAmbiguity() < PhotonConsts.MIN_AMBIGUITY) // remove ambiguous targets
                    .toList();

                    for(PhotonTrackedTarget target : validTargets) {
                        Pose3d cameraPose = new Pose3d();
                        Pose3d targetPose = cameraPose.transformBy(target.getBestCameraToTarget())
                            .transformBy(cameraToRobotTransforms.get(cameras.indexOf(cam))); //I didn't want the index var anymore, idk it looks stupid

                        Translation2d robotToTargetTranslation = targetPose.toPose2d().getTranslation();
                        double dist = robotToTargetTranslation.getNorm();

                        if(dist < minDistance) {
                            minDistance = dist;
                            closestTranslation = robotToTargetTranslation;
                            closestID = target.getFiducialId();
                        }

                    }
            }
        }

        return (closestTranslation != null) ? Optional.of(closestTranslation) : Optional.empty();
    }

    public Optional<Translation2d> getClosestTargetTranslation() {
        return targetFound ? Optional.of(closestTargetTranslation) : Optional.empty();
    }

    private int getClosestValidTargetID() {
        return reefIDs.stream()
                .min(Comparator.comparingInt(id -> (closestTargetTranslation != null)
                        ? (int) closestTargetTranslation.getNorm()
                        : Integer.MAX_VALUE))
                .orElse(-1);
    }

    private void updateEstimationStdDevs(int numTags, double avgDist) {
        curStdDevs = numTags > 1 ? VecBuilder.fill(0.5, 0.5, 1) : VecBuilder.fill(4, 4, 8);
        if(numTags == 1 && avgDist > 4) {//dist in meters
            curStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            curStdDevs = curStdDevs.times(1 + (avgDist * avgDist / 30));
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
}
