package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;



public class DoNothing extends SequentialCommandGroup {

	public DoNothing(SwerveDrive swerve, IntakeArm intakeArm) {

		addCommands(
			new InstantCommand(() -> swerve.ZeroGyro())
			//new InstantCommand(() -> swerve.resetSwerveModuleAngles()) // full robot had this but the swerve implementation I copied did not
		);
	}

}