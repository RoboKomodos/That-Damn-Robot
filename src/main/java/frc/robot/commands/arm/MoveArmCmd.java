// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.utils.Direction;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class MoveArmCmd extends Command {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final ArmSubsystem arm;
	private final Direction direction;

	/**
	 * Creates a new Move Arm Command.
	 *
	 * @param subsystem The arm subsystem used by this command.
	 * @param direction The direction the arm will move.
	 */
	public MoveArmCmd(ArmSubsystem subsystem, Direction direction) {
		arm = subsystem;
		this.direction = direction;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		arm.moveArm(direction);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		arm.stopArm();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
