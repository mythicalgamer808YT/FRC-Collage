package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonConsts;

public class PhotonSubsystem extends SubsystemBase{
    private Matrix<N3,N1> curStdDevs = PhotonConsts.SINGLE_STD_DEVS;

    private List<PhotonCamera> cameras;
    private List<Transform3d> cameraToRobotTransforms;
    private final CommandSwerveDrivetrain drivetrain;

    private List<Integer> reefIDs;//switch to Set if theres a lotta reef ID's
    private Translation2d closestTargetTranslation = null;
    private Rotation2d closestTargetRotation = new Rotation2d();

    private int currentTargetID = -1;
    private int newTargetID = -1;
    private double lastSeenTime = 0;
    private double lastTargetSwitchTime = 0;

    //STD dev for odometry
    private final List<Translation2d> visionTranslationBuffer = new ArrayList<>();
    private final List<Rotation2d> visionRotationBuffer = new ArrayList<>();

    //nga see
    private boolean targetFound = false;

    public PhotonSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.cameraToRobotTransforms = PhotonConsts.CAM_TO_ROBOT_TRANSFORMS;
        this.cameras = PhotonConsts.CAM_NAMES.stream().map(PhotonCamera::new).toList();
        this.drivetrain = drivetrain;

        this.reefIDs = PhotonConsts.VALID_REEF_IDS;
    }

    @Override
    public void periodic() {
        Optional<Translation2d> targetPoseOpt = getRobotToTargetTranslation();

        Pose2d currentOdometryPose = drivetrain.getPose();
        Translation2d odometryTranslation = currentOdometryPose.getTranslation();
        Rotation2d odometryRotation = currentOdometryPose.getRotation();

        if(targetPoseOpt.isPresent()) {
            Translation2d newTargetTranslation = targetPoseOpt.get();
            Rotation2d newTargetRotation = computeTargetRotation(newTargetTranslation);

            double distanceDifference = newTargetTranslation.minus(odometryTranslation.unaryMinus()).getNorm();
            double angleDifference = Math.abs(newTargetRotation.getRadians() - odometryRotation.getRadians());

            if(visionTranslationBuffer.size() >= PhotonConsts.BUFFER_SIZE) {
                visionTranslationBuffer.remove(0);
                visionRotationBuffer.remove(0);
            }
            visionTranslationBuffer.add(newTargetTranslation);
            visionRotationBuffer.add(newTargetRotation);

            double combinedStdDev = computeCombinedStdDev(visionTranslationBuffer, visionRotationBuffer);
            SmartDashboard.putNumber("Combined Std Dev", combinedStdDev);
            
            boolean isStable = (visionTranslationBuffer.size() == PhotonConsts.BUFFER_SIZE) &&
                                (combinedStdDev < PhotonConsts.MAX_COMBINED_STD_DEV);

            double currentTime = Timer.getFPGATimestamp();

            if(isStable) {
                closestTargetTranslation = odometryTranslation.interpolate(newTargetTranslation.unaryMinus(), PhotonConsts.BLEND_FACTOR);
                closestTargetRotation = new Rotation2d(
                    odometryRotation.getRadians() + PhotonConsts.BLEND_FACTOR * (newTargetRotation.getRadians() - odometryRotation.getRadians())
                );
            } else {
                closestTargetTranslation = odometryTranslation.unaryMinus();
                closestTargetRotation = odometryRotation;
            }

            Translation2d flippedTranslation = closestTargetTranslation.unaryMinus();

            if(newTargetID != currentTargetID && isStable) {
                if(currentTargetID == -1 || (currentTime - lastSeenTime) > PhotonConsts.TARGET_SWITCH_DELAY) {
                    drivetrain.resetOdometry(new Pose2d(flippedTranslation, closestTargetRotation));
                    currentTargetID = newTargetID;
                    lastTargetSwitchTime = currentTime;
                }
            }

            lastSeenTime = currentTime;
            targetFound = true;
        } else { // don't see crap so rely on Odometry
            if(Timer.getFPGATimestamp() - lastSeenTime > PhotonConsts.TARGET_LOST_TIMEOUT) {
                closestTargetTranslation = null;
                closestTargetRotation = new Rotation2d();
                targetFound = false;
                currentTargetID = -1;

                visionRotationBuffer.clear();
                visionTranslationBuffer.clear();
            } else {
                double timeSinceLastVision = Timer.getFPGATimestamp() - lastSeenTime;
                double blendFactor = Math.min(1, timeSinceLastVision / 0.2); // the number is the total time before full odometry reliance

                closestTargetTranslation = odometryTranslation.unaryMinus().interpolate(closestTargetTranslation, blendFactor);
                closestTargetRotation = new Rotation2d(
                    odometryRotation.getRadians() + blendFactor * (closestTargetRotation.getRadians() - odometryRotation.getRadians())
                );
            }
        }

        SmartDashboard.putBoolean("Target Found", targetFound);
        SmartDashboard.putNumber("Current Target ID", currentTargetID);
        if(closestTargetTranslation != null) {
            SmartDashboard.putNumber("Closest Target X", closestTargetTranslation.getX());
            SmartDashboard.putNumber("Closest Target Y", closestTargetTranslation.getY());
        }
    }

    public Optional<Translation2d> getRobotToTargetTranslation() {
        double minDistance = Double.MAX_VALUE;
        Translation2d closestTranslation = null;

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
                            .transformBy(cameraToRobotTransforms.get(cameras.indexOf(cam)));
                        
                        Translation2d robotToTargetTranslation = targetPose.toPose2d().getTranslation();
                        
                        double dist = robotToTargetTranslation.getNorm();

                        if(dist < minDistance) {
                            minDistance = dist;
                            closestTranslation = robotToTargetTranslation;
                            newTargetID = target.getFiducialId();
                        }

                    }
            }
        }

        return (closestTranslation != null) ? Optional.of(closestTranslation) : Optional.empty();
    }

    public Rotation2d computeTargetRotation(Translation2d targetTranslation) {
        // Pose2d robotPose = drivetrain.getPose();
        // Translation2d robotTranslation = robotPose.getTranslation();

        double angleToTarget = Math.atan2(
            targetTranslation.getY(),
            targetTranslation.getX()
        );

        return new Rotation2d(angleToTarget);
    }

    private double computeCombinedStdDev(List<Translation2d> translationBuffer, List<Rotation2d> rotationBuffer) {
        if(translationBuffer.size() < 2 || rotationBuffer.size() < 2) return Double.MAX_VALUE;

        double meanX = translationBuffer.stream().mapToDouble(Translation2d::getX).average().orElse(0);
        double meanY = translationBuffer.stream().mapToDouble(Translation2d::getY).average().orElse(0);
        double varianceX = translationBuffer.stream().mapToDouble(t -> Math.pow(t.getX() - meanX, 2)).sum() / translationBuffer.size();
        double varianceY = translationBuffer.stream().mapToDouble(t -> Math.pow(t.getY() - meanY, 2)).sum() / translationBuffer.size();
        double translationStdDev = Math.sqrt(varianceX + varianceY);

        double meanRotation = rotationBuffer.stream().mapToDouble(Rotation2d::getRadians).average().orElse(0);
        double rotationVariance = rotationBuffer.stream().mapToDouble(r -> Math.pow(r.getRadians() - meanRotation, 2)).sum() / rotationBuffer.size();
        double rotationStdDev = Math.sqrt(rotationVariance);

        //tune weights
        return (0.8 * translationStdDev) +
                (0.4 * rotationStdDev);
    }

    public Optional<Translation2d> getClosestTargetTranslation() {
        return targetFound ? Optional.of(closestTargetTranslation) : Optional.empty();
    }

    public Optional<Rotation2d> getClosestTargetRotation() {
        return targetFound ? Optional.of(closestTargetRotation) : Optional.empty();
    }

    // private int getClosestValidTargetID() {
    //     return reefIDs.stream()
    //             .min(Comparator.comparingInt(id -> (closestTargetTranslation != null)
    //                     ? (int) closestTargetTranslation.getNorm()
    //                     : Integer.MAX_VALUE))
    //             .orElse(-1);
    // }

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
