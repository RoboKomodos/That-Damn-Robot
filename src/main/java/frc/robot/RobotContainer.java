// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.arm.MoveArmCmd;
import frc.robot.commands.climber.ToggleClimberCmd;
// import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.Direction;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ClimberSubsystem climber = new ClimberSubsystem();

  private final ArmSubsystem arm = new ArmSubsystem();

  private final IntakeSubsystem intake = new IntakeSubsystem();

  final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   m_driverController.getHID()::getYButtonPressed,
                                                                   m_driverController.getHID()::getAButtonPressed,
                                                                   m_driverController.getHID()::getXButtonPressed,
                                                                   m_driverController.getHID()::getBButtonPressed);
     // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -m_driverController.getRightX(),
        () -> -m_driverController.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRightX() * 0.5);

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  
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
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Swerve/Position Triggers
    m_driverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //m_driverController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    m_driverController.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
                              
    // Climber Trigger
    //m_driverController.y().onTrue(climber.activateClimber());
    m_driverController.y().onTrue(new ToggleClimberCmd(climber));

    // Arm Triggers
    // m_driverController.rightTrigger().onTrue(arm.raiseArm());
    // m_driverController.rightTrigger().onFalse(arm.stopArm());
    // m_driverController.leftTrigger().onTrue(arm.lowerArm());
    // m_driverController.leftTrigger().onFalse(arm.stopArm());
    m_driverController.rightTrigger().onTrue(new MoveArmCmd(arm, Direction.UP));
    m_driverController.leftTrigger().onTrue(new MoveArmCmd(arm, Direction.DOWN));

    // Intake Triggers
    m_driverController.b().onTrue(intake.toggleFlyWheel());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode(){}

  public void setMotorBrake(boolean brake){
    drivebase.setMotorBrake(brake);
  }

  // public 
}
