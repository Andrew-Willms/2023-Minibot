package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;



// An example command that uses an example subsystem.
public class Pickup extends CommandBase {

	private final IntakeArm IntakeArm;
	private final IntakeWheels IntakeWheels;

	public Pickup(IntakeArm intakeArm, IntakeWheels intakeWheels) {

		IntakeArm = intakeArm;
		IntakeWheels = intakeWheels;
		addRequirements(intakeArm, intakeArm);

		IntakeArm.SetPosition(Constants.IntakeArm.Position.Pickup);
		IntakeWheels.SetTarget(Constants.IntakeWheels.Target.Pickup);
	}

	@Override
	public boolean isFinished() {
		return true;
	}

}