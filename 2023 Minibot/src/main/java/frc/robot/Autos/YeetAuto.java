package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Stop;
import frc.robot.commands.SuckWhileUp;
import frc.robot.commands.Yeet;
import frc.robot.subsystems.*;



public class YeetAuto extends SequentialCommandGroup {

	public YeetAuto(SwerveDrive swerve, IntakeArm intakeArm, IntakeWheels IntakeWheels) {

		addCommands(
			new InstantCommand(() -> swerve.ZeroGyro()),
			new InstantCommand(() -> swerve.resetSwerveModuleAngles()),
			new SuckWhileUp(IntakeWheels),
			new WaitCommand(0.5),
			new Yeet(intakeArm, IntakeWheels),
			new WaitCommand(1),
			new Stop(IntakeWheels)
		);
	}

}