package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.automations.*;
import frc.robot.automations.driveAutomizations.TurnToAngleWhileDriving;
import frc.robot.commands.driveCommands.TeleopSwerve;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public static double translationVal;
    public static double strafeVal;

    /* Controllers */
    public static final XboxController xboxController = new XboxController(0);
    public static final CommandXboxController commandXBoxController = new CommandXboxController(0);
    private final static Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */

    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton shootUpButton = new JoystickButton(driver, XboxController.Button.kA.value);

    POVButton d_Uppov = new POVButton(driver, 0);
    POVButton d_Rightpov = new POVButton(driver, 90);
    POVButton d_Downpov = new POVButton(driver, 180);
    POVButton d_Leftpov = new POVButton(driver, 270);

    // private final JoystickButton IntakeEnableCommand = new JoystickButton(driver,
    // XboxController.Button.kRightBumper.value);
    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    public static final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    public static final TransportationSubsystem m_TransportationSubsystem = new TransportationSubsystem();
    public static final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();

    private final SendableChooser<Command> autoChooser;
    static private Alliance robotAlliance = Alliance.Blue;

    public static Alliance getAlliance() {
        return robotAlliance;
    }

    /**
     * q+
     * `
     * The container
     * for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Build an auto chooser. This will use Commands.none() as the default option.
        // Command intake = Commands.run(() -> m_TransportationSubsystem.setSpeed(0.6,
        // 0.6, 1));

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        registerCommands();
        configureButtonBindings();
        setDefaultCommands();
        autoChooser = AutoBuilder.buildAutoChooser();

        Shuffleboard.getTab("Robot")
                .add("Auto", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        // Shooter Triggers
        commandXBoxController.a().toggleOnTrue(m_ShooterSubsystem.shootUpCommand());
        commandXBoxController.b().toggleOnTrue(m_ShooterSubsystem.shooterDownCommand());

        // Transportation Triggers
        commandXBoxController.rightBumper()
                .toggleOnTrue(
                        m_TransportationSubsystem.transportUpCommand()
                                .alongWith(
                                        new TurnToAngleWhileDriving(
                                                s_Swerve,
                                                () -> Robot.getRobotToNoteYaw() + s_Swerve.getHeading().getDegrees(),
                                                () -> -driver.getRawAxis(translationAxis),
                                                () -> -driver.getRawAxis(strafeAxis),
                                                () -> -driver.getRawAxis(rotationAxis))));

        commandXBoxController.x().toggleOnTrue(m_TransportationSubsystem.transportDownCommand());

        // Climb Triggers
        commandXBoxController.rightTrigger()
                .onTrue(m_ClimbSubsystem.setSpeedCommand(-RobotContainer.xboxController.getRightTriggerAxis(),
                        RobotContainer.xboxController.getRightTriggerAxis()));

        // Utilities
    }

    private void setDefaultCommands() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()));
        m_ShooterSubsystem.setDefaultCommand(m_ShooterSubsystem.stopMotorsCommand());
        m_TransportationSubsystem.setDefaultCommand(m_TransportationSubsystem.stopMotorsCommand());
        m_ClimbSubsystem.setDefaultCommand(m_ClimbSubsystem.stopMotorsCommand());
    }

    // IntakeEnableCommand.onTrue(Commands.sequence(intake));

    private void registerCommands() {
        NamedCommands.registerCommand("HighShooter", new enableHighShooter());
        NamedCommands.registerCommand("LowShooter", new enableLowShooter());
        NamedCommands.registerCommand("StopShooter", new disableShooter());
        NamedCommands.registerCommand("StartTransportation", new enableForwardTransportation());
        NamedCommands.registerCommand("StopTransportation", new disableTransportation());
        NamedCommands.registerCommand("StartBackTransportation", new enableBackTransportation());
        NamedCommands.registerCommand("StartHighSpeedTransportation", new enableForwardTransportationHighSpeed());

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
