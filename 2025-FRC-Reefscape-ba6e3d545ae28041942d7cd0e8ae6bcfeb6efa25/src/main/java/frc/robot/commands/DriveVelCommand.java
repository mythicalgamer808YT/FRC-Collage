// package frc.robot.commands;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.lib.util.Utilities;
// import frc.robot.Constants.DriveVelConsts;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.DriveVelSubsystem;

// public class DriveVelCommand {

//     DriveVelSubsystem driveSubsystem;
//     CommandSwerveDrivetrain drivetrain;
//     SwerveRequest.FieldCentric drive;

//     private double[] driveVel;

//     public DriveVelCommand(DriveVelSubsystem driveSubsystem, CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive) {
//         this.driveSubsystem = driveSubsystem;
//         this.drivetrain = drivetrain;
//         this.drive = drive;
//     }

//     public Command setDrive() {
//         driveVel = driveSubsystem.getDriveVel();

//         return drivetrain.applyRequest(() ->
//             drive.withVelocityX(Utilities.polynomialAccleration(driveVel[0]) * -DriveVelConsts.MAXSPEED * 0.4) // Drive forward with negative Y (forward)
//                 .withVelocityY(Utilities.polynomialAccleration(driveVel[1]) * -DriveVelConsts.MAXSPEED * 0.4) // Drive left with negative X (left)
//                 .withRotationalRate(Utilities.polynomialAccleration(driveVel[2]) * -DriveVelConsts.MAX_ANGULAR_RATE * 0.4));// Drive counterclockwise with negativeX (left)
//     }

// }