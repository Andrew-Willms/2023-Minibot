package frc.robot.Autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.DriveForwards;
import frc.robot.commands.Stop;
import frc.robot.commands.StopDriving;
import frc.robot.commands.SuckWhileUp;
import frc.robot.commands.Yeet;
import frc.robot.subsystems.*;



public class YeetDriveForwards extends SequentialCommandGroup {

    private final SwerveDrive SwerveDrive;
    private final IntakeArm IntakeArm;
    private final IntakeWheels IntakeWheels;

	public YeetDriveForwards(SwerveDrive swerveDrive, IntakeArm intakeArm, IntakeWheels intakeWheels) {

        SwerveDrive = swerveDrive;
        IntakeArm = intakeArm;
        IntakeWheels = intakeWheels;

		addCommands(
			new InstantCommand(() -> SwerveDrive.ZeroGyro()),
			new InstantCommand(() -> SwerveDrive.resetSwerveModuleAngles()),
			new SuckWhileUp(IntakeWheels),
			new WaitCommand(0.5),
			new Yeet(IntakeArm, IntakeWheels),
			new WaitCommand(1),
			new Stop(IntakeWheels),
            new DriveForwards(swerveDrive, Constants.Autos.YeetDriveForwards.Translation, 0),
            new WaitCommand(Constants.Autos.YeetDriveForwards.DriveForwardsDuration),
            new StopDriving(swerveDrive)
		);
	}

}