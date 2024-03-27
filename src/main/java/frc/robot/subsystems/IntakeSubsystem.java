package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class IntakeSubsystem extends SubsystemBase{

    private final CANSparkMax flyWheelT;
    private final CANSparkMax flyWheelB;
    private final SparkPIDController flyWheelPIDControllerT;
    private final SparkPIDController flyWheelPIDControllerB;

    private final WPI_TalonSRX intakeRoller = new WPI_TalonSRX(Constants.IntakeConstants.CAN_ID_INTAKE_MOTOR);

    private boolean isRunning = false;

    public IntakeSubsystem(){
        // Motor objects and PID controllers
        flyWheelT = new CANSparkMax(Constants.IntakeConstants.CAN_ID_FLYWHEEL_MOTOR_T, MotorType.kBrushless);
        flyWheelB = new CANSparkMax(Constants.IntakeConstants.CAN_ID_FLYWHEEL_MOTOR_B, MotorType.kBrushless);

        flyWheelPIDControllerT = flyWheelT.getPIDController();
        flyWheelPIDControllerB = flyWheelB.getPIDController();



        flyWheelPIDControllerT.setP(Constants.IntakeConstants.FLYWHEEL_P);
        flyWheelPIDControllerT.setI(Constants.IntakeConstants.FLYWHEEL_I);
        flyWheelPIDControllerT.setD(Constants.IntakeConstants.FLYWHEEL_D);
        flyWheelPIDControllerT.setIZone(Constants.IntakeConstants.FLYWHEEL_IZONE);
        flyWheelPIDControllerT.setFF(Constants.IntakeConstants.FLYWHEEL_FF);
        flyWheelPIDControllerT.setOutputRange(Constants.IntakeConstants.FLYWHEEL_MIN, Constants.IntakeConstants.FLYWHEEL_MAX);

        flyWheelPIDControllerB.setP(Constants.IntakeConstants.FLYWHEEL_P);
        flyWheelPIDControllerB.setI(Constants.IntakeConstants.FLYWHEEL_I);
        flyWheelPIDControllerB.setD(Constants.IntakeConstants.FLYWHEEL_D);
        flyWheelPIDControllerB.setIZone(Constants.IntakeConstants.FLYWHEEL_IZONE);
        flyWheelPIDControllerB.setFF(Constants.IntakeConstants.FLYWHEEL_FF);
        flyWheelPIDControllerB.setOutputRange(Constants.IntakeConstants.FLYWHEEL_MIN, Constants.IntakeConstants.FLYWHEEL_MAX);

        // intakeRoller declared outside of constructor


        flyWheelT.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 20);
        flyWheelT.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 40);
        flyWheelT.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 40);
        flyWheelT.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 100);
        flyWheelT.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 20);
        flyWheelT.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 400);
        flyWheelT.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 400);

        flyWheelB.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 20);
        flyWheelB.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 40);
        flyWheelB.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 40);
        flyWheelB.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 100);
        flyWheelB.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 20);
        flyWheelB.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 400);
        flyWheelB.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 400);

        flyWheelB.follow(flyWheelT);
    }

    public Command toggleIntake(){
        if(intakeRoller.getMotorOutputVoltage() == 0.0){
            return run(() -> {
                setIntakeVoltage(Constants.IntakeConstants.INTAKE_MAX_VOLTAGE);
            });
        } else {
            return run(() -> {
                setIntakeVoltage(0.0);
            });
        }
    }

    public void setIntakeVoltage(double voltage){
        intakeRoller.setVoltage(voltage);
    }

    public double getIntakeVoltage() {
        return intakeRoller.getMotorOutputVoltage();
    }

    public void moveFlyWheel(double speed) {
        flyWheelT.set(speed);
        isRunning = true;
    }

    public void moveFlyWheel() {
        moveFlyWheel(Constants.IntakeConstants.FLYWHEEL_SPEED);
    }

    public void stopFlyWheel() {
        flyWheelT.stopMotor();
        isRunning = false;
    }

}