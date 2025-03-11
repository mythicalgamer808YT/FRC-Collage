package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib.util.Utilities;

import frc.robot.States.ElevatorStates;
import frc.robot.States.IndexStates;
import frc.robot.commands.ElevatorCommandHandler2;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.InnerElevatorSubsystem;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

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
    private final PrimaryElevatorSubsystem s_primaryElevatorSubsystem = new PrimaryElevatorSubsystem();
    private final InnerElevatorSubsystem s_innerElevatorSubsystem = new InnerElevatorSubsystem();
    private final IntakeArmSubsystem s_intakeArmSubsystem = new IntakeArmSubsystem();
    //** Command Handlers **//
    private final ElevatorCommandHandler2 ch_elevatorCommandHandler = new ElevatorCommandHandler2(s_innerElevatorSubsystem, s_primaryElevatorSubsystem, s_intakeArmSubsystem);
    private final IntakeCommand ch_intakeCommand = new IntakeCommand(s_intakeArmSubsystem);
    //private final DriveVelCommand ch_driveVelCommand = s_driveVelSubsystem.getCommands();

    //** Commands **//
    private final SequentialCommandGroup score = new SequentialCommandGroup(
        new InstantCommand(() -> s_intakeArmSubsystem.changeAngle(-0.3), s_intakeArmSubsystem),
        new ParallelRaceGroup(
            new WaitUntilCommand(s_intakeArmSubsystem::atAngle),
            new WaitCommand(0.4)//time out
        ),
        ch_intakeCommand.setIntakeState(IndexStates.INTAKE),
        new WaitCommand(0.3),
        new InstantCommand(() -> s_intakeArmSubsystem.changeAngle(0.3), s_intakeArmSubsystem),
        ch_elevatorCommandHandler.setElevatorState(ElevatorStates.PRE_INTAKE),
        ch_intakeCommand.setIntakeState(IndexStates.STOP)
    );

    private SequentialCommandGroup c_preIntakeToIntake = new SequentialCommandGroup(
        new InstantCommand(() -> s_intakeArmSubsystem.setAngle(ElevatorStates.INTAKE.armAngle)),
        new InstantCommand(() -> s_primaryElevatorSubsystem.setPrimaryElevatorHeight(ElevatorStates.INTAKE.primaryHeight)),
        new WaitCommand(0.5),
        new InstantCommand(() -> s_innerElevatorSubsystem.setInnerElevatorHeight(ElevatorStates.INTAKE.innerHeight))
    );

    public RobotContainer() {
        NamedCommands.registerCommand("Score", score);
        NamedCommands.registerCommand("L4", ch_elevatorCommandHandler.setElevatorState(ElevatorStates.L4));
        NamedCommands.registerCommand("Pre intake", ch_elevatorCommandHandler.setElevatorState(ElevatorStates.PRE_INTAKE));
        NamedCommands.registerCommand("intake", new SequentialCommandGroup(c_preIntakeToIntake, ch_intakeCommand.setIntakeState(IndexStates.INTAKE),
                                                                   new WaitCommand(0.5), ch_elevatorCommandHandler.setElevatorState(ElevatorStates.PRE_INTAKE)));
        NamedCommands.registerCommand("Intake", ch_intakeCommand.setIntakeState(IndexStates.INTAKE));
        NamedCommands.registerCommand("Stop", ch_intakeCommand.setIntakeState(IndexStates.STOP));

        m_chooser = AutoBuilder.buildAutoChooser("Blue Bottom Corner");
        SmartDashboard.putData(m_chooser);
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

        // reset the field-centric heading on left bumper press
        driver0.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    private void configureDriver1Commands() {
        driver1.pov(0).toggleOnTrue(ch_elevatorCommandHandler.setElevatorState(ElevatorStates.L4));
        //driver1.pov(90).toggleOnTrue(c_preIntakeToIntake);
        driver1.pov(180).toggleOnTrue(ch_elevatorCommandHandler.setElevatorState(ElevatorStates.L3));
        driver1.pov(270).toggleOnTrue(ch_elevatorCommandHandler.setElevatorState(ElevatorStates.PRE_INTAKE));

        driver1.a().toggleOnTrue(ch_elevatorCommandHandler.setElevatorState(ElevatorStates.STARTING_POSITION));

        driver1.rightTrigger().onTrue(ch_intakeCommand.setIntakeState(IndexStates.INTAKE));
        driver1.rightTrigger().onFalse(ch_intakeCommand.setIntakeState(IndexStates.STOP));


        driver1.leftTrigger().onTrue(ch_intakeCommand.setIntakeState(IndexStates.FEED_OUT));
        driver1.leftTrigger().onFalse(ch_intakeCommand.setIntakeState(IndexStates.STOP));

        //driver1.y().onTrue(score);

        driver1.b().toggleOnTrue(ch_intakeCommand.setIntakeState(IndexStates.HOLD));
        driver1.b().toggleOnFalse(ch_intakeCommand.setIntakeState(IndexStates.STOP));
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}