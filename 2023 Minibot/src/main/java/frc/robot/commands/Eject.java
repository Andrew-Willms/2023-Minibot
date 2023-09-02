package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;



public class Eject extends CommandBase {

	private final IntakeArm IntakeArm;
	private final IntakeWheels IntakeWheels;

	public Eject(IntakeArm intakeArm, IntakeWheels intakeWheels) {

		IntakeArm = intakeArm;
		IntakeWheels = intakeWheels;
		addRequirements(intakeArm, intakeWheels);
	}

	@Override
	public void initialize() {

		IntakeArm.SetPosition(Constants.IntakeArm.Position.Carry);
		IntakeWheels.SetPower(Constants.IntakeWheels.Power.Edject.Value);

		SmartDashboard.putString("Command", "Eject");
	}

	@Override
	public void execute() { }

	@Override
	public boolean isFinished() {
		return true;
	}

}