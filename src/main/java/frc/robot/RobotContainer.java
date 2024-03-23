package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.automations.TestShooter;
import frc.robot.automations.driveAutomizations.AimAssist;
import frc.robot.autos.OneNote;
import frc.robot.autos.TransportAllTime;
import frc.robot.autos.TransportDown;
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

	/* Controllers */
	public static final XboxController xboxController = new XboxController(0);
	public static final CommandXboxController commandXBoxController = new CommandXboxController(0);
	private final static Joystick driver = new Joystick(0);

	/* Drive Controls */
	private final int translationAxis = XboxController.Axis.kLeftY.value;
	private final int strafeAxis = XboxController.Axis.kLeftX.value;
	private final int rotationAxis = XboxController.Axis.kRightX.value;

	/* Driver Buttons */

	private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
	private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

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
		// Another option that allows you to specify the default auto by its name
		// autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
		configureButtonBindings();
		setDefaultCommands();
		registerCommand();
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
		commandXBoxController.a().whileTrue(m_ShooterSubsystem.shootUpCommand().alongWith(
				new WaitCommand(0.5).andThen(m_TransportationSubsystem.transportUpCommand())));
		commandXBoxController.b().whileTrue(m_ShooterSubsystem.shooterDownCommand().alongWith(
				new WaitCommand(0.5).andThen(m_TransportationSubsystem.transportUpCommand())));

		// Transportation Triggers
		commandXBoxController.rightBumper().toggleOnTrue(
				m_TransportationSubsystem.transportUpCommand());
		commandXBoxController.x().toggleOnTrue(m_TransportationSubsystem.transportDownCommand());

		// Climb Triggers
		// commandXBoxController.rightTrigger().whileTrue(m_ClimbSubsystem.setSpeedCommand());

		// Aim Assist Triggers
		commandXBoxController.leftTrigger().whileTrue(new AimAssist(
				s_Swerve,
				() -> -driver.getRawAxis(translationAxis),
				() -> -driver.getRawAxis(strafeAxis),
				() -> -driver.getRawAxis(rotationAxis))
				.alongWith(m_TransportationSubsystem.transportUpCommand()));

		// Test Triggers
		// commandXBoxController.back().toggleOnTrue(s_Swerve.crossWheelsCommand());
	}

	private void setDefaultCommands() {
		s_Swerve.setDefaultCommand(new TeleopSwerve(
				s_Swerve,
				() -> -driver.getRawAxis(translationAxis),
				() -> -driver.getRawAxis(strafeAxis),
				() -> -driver.getRawAxis(rotationAxis),
				() -> robotCentric.getAsBoolean()));

		m_ShooterSubsystem.setDefaultCommand(m_ShooterSubsystem.stopMotorsCommand());

		m_TransportationSubsystem.setDefaultCommand(m_TransportationSubsystem.stopMotorsCommand());

		// m_ClimbSubsystem.setDefaultCommand(m_ClimbSubsystem.stopMotorsCommand());

	}

	public void registerCommand() {
		NamedCommands.registerCommand("ShooterUpDown",
				m_TransportationSubsystem.transportDowmAutoCommand(0.3)

						.andThen(m_ShooterSubsystem.shootUpAutoCommand(2)

								.alongWith(new WaitCommand(0.5)
										.andThen(m_TransportationSubsystem
												.transportUpAutoCommand(2)))));
		NamedCommands.registerCommand("StopShooter1",
				m_ShooterSubsystem.stopMotorsCommand());
		NamedCommands.registerCommand("TransportUp",
				m_TransportationSubsystem.transportUpAutoCommand(3));
		NamedCommands.registerCommand("Shooter",
				m_ShooterSubsystem.shootUpAutoCommand(2)
						.alongWith(new WaitCommand(0.6)
								.andThen(m_TransportationSubsystem.transportUpAutoCommand(1))));
		// NamedCommands.registerCommand("TransportDown",
		// m_TransportationSubsystem.transportDowmAutoCommand(0.3));
		/*
		 * p
		 * NamedCommands.registerCommand("AutoAimAssist", new AimAssistAutonomus(
		 * s_Swerve,
		 * () -> -driver.getRawAxis(translationAxis),
		 * () -> -driver.getRawAxis(strafeAxis),
		 * () -> -driver.getRawAxis(rotationAxis)));
		 */
		NamedCommands.registerCommand(
				"AimAtNote",
				new InstantCommand(
						() -> PPHolonomicDriveController
								.setRotationTargetOverride(s_Swerve::getRotationTargetOverride)));
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
