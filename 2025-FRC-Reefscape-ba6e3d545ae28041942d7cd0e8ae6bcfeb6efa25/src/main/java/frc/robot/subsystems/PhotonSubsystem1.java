package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.States.PhotonStates;
import frc.robot.generated.TunerConstants;

public class PhotonSubsystem1 extends SubsystemBase{
    private PhotonCamera camera;
    private List<PhotonPipelineResult> results;
    private PhotonPipelineResult pipelineResult;
    private Transform3d sigma;

    private Optional<Double> targetYaw;

    private double targetRange;
    //private double targetYaw;
    private boolean targetSeen;
    private double targetID; 
    private boolean rotate;

    private double cameraHeight;
    private double cameraPitch;

    private double currentTime = Timer.getFPGATimestamp();
    private double lastTime = currentTime;

    public PhotonSubsystem1(String cameraName, double cameraHeight, double cameraPitch) {
        camera = new PhotonCamera(cameraName);
        this.cameraHeight = cameraHeight;
        this.cameraPitch = cameraPitch;
        this.rotate = true;
        this.targetYaw = Optional.empty();
    }

    @Override 
    public void periodic(){
        results = camera.getAllUnreadResults();
        targetSeen = false;
        currentTime = Timer.getFPGATimestamp();
    
        // check for results and place default values if there are none 
        if(results.isEmpty()) {
            //targetYaw = 0;
            if(currentTime - lastTime > .5) {
                rotate  = true;
                lastTime = currentTime;
            }
            targetRange = 0;
            return;
        } 

        // get most recent result
        pipelineResult = results.get(results.size() - 1);
        if(pipelineResult.hasTargets()) {
            for(PhotonTrackedTarget target: pipelineResult.getTargets()) {
                
                // calculate key data 
                targetSeen = true;
                //targetYaw = target.getYaw();
                targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                        cameraHeight, 
                        Units.inchesToMeters(8.75), 
                        cameraPitch, 
                        Units.degreesToRadians(target.getPitch()));
                sigma = target.getBestCameraToTarget();//.plus(new Transform3d(-0.1956816, -0.28160218, 0, new Rotation3d()));
                
                if(targetYaw.isEmpty() || currentTime - lastTime < 0.2) {
                    targetYaw = Optional.of(target.getYaw());
                    rotate = true;
                } else {
                    rotate = false;
                }

                //targetYaw = Math.atan(sigma.getY() / sigma.getX());
            }
        }
        setSmartdashboardData();
    }   

    public Optional<Double> getYaw() {
        return targetYaw;
    }
    public boolean canRotate() {
        return rotate;
    }

    public void resetYaw() {
        targetYaw = Optional.empty();
    }

    public Transform3d getDistances() {
        return sigma; 
    }

    public double getDistance() {
        return targetRange;
    }

    // public double getYaw() {
    //     return targetYaw;
    // }
    // calculate the turn outpu

    private void setSmartdashboardData() {
        // return motor output data 
        

        // return calculated tag(target) data
        SmartDashboard.putNumber(camera.getName() + " target yaw", targetYaw.get());
        SmartDashboard.putNumber(camera.getName() + " target range", targetRange);
        SmartDashboard.putBoolean(camera.getName() + " target seen", targetSeen);
    
    }
}