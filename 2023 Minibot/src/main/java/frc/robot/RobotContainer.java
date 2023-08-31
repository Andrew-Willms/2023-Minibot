// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.autos.BaseAuto;
//import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final Joystick DriverController = new Joystick(OperatorConstants.DriverControllerPort);

	// Driver Analog Controls
	private final int TranslationAxisID = XboxController.Axis.kLeftY.value;
	private final int StrafeAxisID = XboxController.Axis.kLeftX.value;
	private final int CCWRotationAxisID = XboxController.Axis.kLeftTrigger.value;
	private final int CWRotationAxisID = XboxController.Axis.kRightTrigger.value;
	private final int IntakeWheelsForwardID = XboxController.Axis.kRightX.value;

	// Driver Buttons
	private final JoystickButton ZeroGyroButton = new JoystickButton(DriverController, XboxController.Button.kY.value);
	private final JoystickButton RobotCentricButton = new JoystickButton(DriverController, XboxController.Button.kLeftBumper.value);
	private final JoystickButton PickupPositionButton = new JoystickButton(DriverController, XboxController.Button.kY.value);
	private final JoystickButton CarryCubePositionButton = new JoystickButton(DriverController, XboxController.Button.kA.value);
	private final JoystickButton CarryConePositionButton = new JoystickButton(DriverController, XboxController.Button.kX.value);

	// Subsystems
	private final SwerveDrive SwerveDrive = new SwerveDrive();
	private final IntakeWheels Intake = new IntakeWheels();

	// The container for the robot. Contains subsystems, OI devices, and commands.
	public RobotContainer() {

		SwerveDrive.setDefaultCommand(
			new TeleopSwerve(
				SwerveDrive,
				() -> -DriverController.getRawAxis(TranslationAxisID),
				() -> -DriverController.getRawAxis(StrafeAxisID),
				() -> -DriverController.getRawAxis(CCWRotationAxisID),
				() -> -DriverController.getRawAxis(CWRotationAxisID),
				() -> RobotCentricButton.getAsBoolean()));

		Intake.setDefaultCommand(new TeleopIntakeWheels);

		ConfigureBindings();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
	 * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void ConfigureBindings() {

		// Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed, cancelling on release.
		//DriverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

		ZeroGyroButton.onTrue(new InstantCommand(() -> SwerveDrive.ZeroGyro()));

		DriverController.
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command GetAutonomousCommand() {
		// An example command will be run in autonomous
		return Autos.exampleAuto(m_exampleSubsystem);
	}

}