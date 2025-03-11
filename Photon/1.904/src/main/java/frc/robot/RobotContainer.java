package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib.util.Utilities;
import frc.robot.commands.PhotonCommand2;
import frc.robot.commands.positionRelativeToAprilTag;
import frc.robot.subsystems.PhotonSubsystem1;
import frc.robot.subsystems.PhotonSubsystem2;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;


//overall structure of the robot here, no real robot logic
//subsystems, commands, and triggermappings, respectively
public class RobotContainer {
    //generated swerve
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    //bindings and all for swerve control
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController driver0 = new CommandXboxController(0);
    private final CommandXboxController driver1 = new CommandXboxController(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    //put choreo
    //private final AutoChooser autoChooser = new AutoChooser();
    private SendableChooser<Command> m_chooser;

    //** Subsystems **//
    // private final PhotonSubsystem1 sigma = new PhotonSubsystem1("gray_photon_camera", Units.inchesToMeters(10), 0);
    // private final PhotonSubsystem1 graySigma = new PhotonSubsystem1("blue_photon_camera",Units.inchesToMeters(10), 0);
    // private final positionRelativeToAprilTag spot = new positionRelativeToAprilTag(graySigma, drivetrain);
    // private final positionRelativeToAprilTag spot1 = new positionRelativeToAprilTag(sigma, drivetrain);
   // private final DriveVelSubsystem s_driveVelSubsystem = new DriveVelSubsystem(driver0, drivetrain, drive);

    //** Command Handlers **//

    private final PhotonSubsystem2 photonSubsystem = new PhotonSubsystem2();

    public RobotContainer() {
        configureDriveBindings();
        configureDriver1Commands();
    }

    private void configureDriveBindings() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-Utilities.polynomialAccleration(driver0.getLeftY()) * MaxSpeed * 0.4) // Drive forward with negative Y (forward)
                    .withVelocityY(-Utilities.polynomialAccleration(driver0.getLeftX()) * MaxSpeed * 0.4) // Drive left with negative X (left)
                    .withRotationalRate(-Utilities.polynomialAccleration(driver0.getRightX()) * MaxAngularRate * 0.4) // Drive counterclockwise with negative X (left)
            )
        );

        driver0.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver0.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver0.getLeftY(), -driver0.getLeftX()))));

        driver0.x().whileTrue(new PhotonCommand2(photonSubsystem, drivetrain, true));
        driver0.y().whileTrue(new PhotonCommand2(photonSubsystem, drivetrain, false));
        // reset the field-centric heading on left bumper press
        driver0.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    private void configureDriver1Commands() {

    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}