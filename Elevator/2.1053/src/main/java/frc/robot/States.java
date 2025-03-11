package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.Utilities;
import frc.robot.States.ElevatorStates;
import frc.robot.Constants.Photon;

public class States {
        public enum PhotonStates {
        driveTag4(Photon.tag4.targetID ,Photon.tag4.distance0, Photon.tag4.angle0, Photon.tag4.tagHeight);

        public final int id;
        public final double distance;
        public final double angle; 
        public final double height;

        PhotonStates(int id, double distance, double angle, double height) {
            this.id = id;
            this.distance = distance;
            this.angle = angle;
            this.height = height;
        }
    }

    public enum ElevatorStates {
        MIN(0, 3.211, 55),
        MAX(112 , 26,220),
        STARTING_POSITION(0.01, 23,71),
        PRE_INTAKE(61.55, 23.5, 90),
        INTAKE(61.55, 21, 61),

        L2(1,7.416,203),
        L3(23.9, 23.5, 200),
        L4(108.5, 23.5, 210),
        CRITICAL_POINT(0,20.3, 80);

        public final double primaryHeight;
        public final double innerHeight;
        public final double armAngle;

        ElevatorStates(double primaryHeight, double innerHeight, double intakeArmAngle) {
            this.primaryHeight = primaryHeight;
            this.innerHeight = innerHeight;
            this.armAngle = intakeArmAngle;
        }

        ElevatorStates() {
            this.primaryHeight = 0;
            this.innerHeight = 0;
            this.armAngle = 0;
        }
    }

    public enum IndexStates {
        // allow for easy changing of elevator states 
        STOP(0),
        INTAKE(1),
        FEED_OUT(-1),
        HOLD(0.4);

        public final double speed;

        IndexStates(double speed) {
            this.speed = speed;
        }
    }

    public enum ClimbStates {
        STOP(0),
        CLIMB(1),
        DROP(-1);

        public final double speed;

        ClimbStates(double speed) {
            this.speed = speed;
        }
    }
}