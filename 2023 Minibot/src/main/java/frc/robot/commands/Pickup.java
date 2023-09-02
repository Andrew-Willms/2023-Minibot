package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
		addRequirements(intakeArm, intakeWheels);
	}

	@Override
	public void initialize() {

		IntakeArm.SetPosition(Constants.IntakeArm.Position.Pickup);
		IntakeWheels.SetPower(Constants.IntakeWheels.Power.Pickup.Value);

		SmartDashboard.putString("Command", "Pickup");
	}

	@Override
	public void execute() { }

	@Override
	public boolean isFinished() {
		return true;
	}

}