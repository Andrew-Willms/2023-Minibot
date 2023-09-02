package frc.robot.Autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.SwerveDrive;




public class DriveForward extends SequentialCommandGroup {

	
	public DriveForward(SwerveDrive swerveDrive, IntakeArm intakeArm, IntakeWheels intakeWheels) {

		PathPlannerTrajectory move = PathPlanner.loadPath("Move Out Of Zone", new PathConstraints(2, 2));

		addCommands(
			new InstantCommand(() -> swerveDrive.ZeroGyro()),
			new InstantCommand(() -> swerveDrive.resetSwerveModuleAngles()),
			new ParallelCommandGroup(swerveDrive.followTrajectoryCommand(move, true))
		);

	}

}