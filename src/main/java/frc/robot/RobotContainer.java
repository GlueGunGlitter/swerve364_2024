package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.automations.*;

import frc.robot.autos.*;
import frc.robot.commands.*;
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
    public static final ShooterCommand m_ShooterCommand = new ShooterCommand();
    public static final TransportationSubsystem m_TransportationSubsystem = new TransportationSubsystem();
    public static final TransportationCommand m_TransportationCommand = new TransportationCommand();
    public static final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
    public static final ClimbCommand m_ClimbCommand = new ClimbCommand();

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

        autoChooser = AutoBuilder.buildAutoChooser();

        Shuffleboard.getTab("Robot")
                .add("Auto", autoChooser);
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()));

        // Configure the button bindings

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
