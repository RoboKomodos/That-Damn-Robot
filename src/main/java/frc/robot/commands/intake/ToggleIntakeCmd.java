package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntakeCmd extends Command {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final IntakeSubsystem intake;

	/**
	 * Creates a new Toggle Intake Command.
	 *
	 * @param subsystem The arm subsystem used by this command.
	 */
	public ToggleIntakeCmd(IntakeSubsystem subsystem) {
		intake = subsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	    if(intake.getIntakeVoltage() == 0.0){
            intake.setIntakeVoltage(Constants.IntakeConstants.INTAKE_MAX_VOLTAGE);
        } else {
            intake.setIntakeVoltage(0.0);
        }
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
