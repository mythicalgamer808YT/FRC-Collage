// package frc.robot.subsystems;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.commands.DriveVelCommand;


// public class DriveVelSubsystem extends SubsystemBase {
//     private double driveVelX;
//     private double driveVelY;

//     private double driveAgl;

//     private CommandXboxController driver0;

//     private DriveVelCommand commands;

//     public DriveVelSubsystem(CommandXboxController driver, CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive) {
//         driveVelX = 0;
//         driveVelY = 0;
//         driveAgl = 0;

//         driver0 = driver;
//         commands = new DriveVelCommand(this, drivetrain, drive);
//     }

//     @Override
//     public void periodic() {
        
//         setDriveVel(driver0.getLeftX(), driver0.getLeftY());
//         driveAgl = driver0.getRightX();
//     }

//     public void setDriveVel(double X, double Y) {
//         driveVelX = X;
//         driveVelY = Y;
//     }

//     //[X, Y]^T
//     public double[] getDriveVel() {
//         return new double[]{driveVelX, driveVelY, driveAgl};
//     }

//     public DriveVelCommand getCommands() {
//         return commands;
//     }

// }