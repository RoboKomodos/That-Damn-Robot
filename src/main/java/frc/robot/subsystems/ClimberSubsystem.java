package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.utils.Direction;

public class ClimberSubsystem extends SubsystemBase{
    private final CANSparkMax climberMotor1;
    private final CANSparkMax climberMotor2;
    private final SparkPIDController climberPIDController1;
    private final SparkPIDController climberPIDController2;
    private final RelativeEncoder climberEncoder1;
    private final RelativeEncoder climberEncoder2;

    private boolean isRaised = true;

    public ClimberSubsystem(){
        // Motor objects, PID controllers, and encoders
        climberMotor1 = new CANSparkMax(Constants.ClimberConstants.CAN_ID_CLIMBER_MOTOR_1, MotorType.kBrushless);
        climberMotor2 = new CANSparkMax(Constants.ClimberConstants.CAN_ID_CLIMBER_MOTOR_2, MotorType.kBrushless);

        climberPIDController1 = climberMotor1.getPIDController();
        climberPIDController2 = climberMotor2.getPIDController();

        climberEncoder1 = climberMotor1.getEncoder();
        climberEncoder2 = climberMotor2.getEncoder();

        // Set PID constants
        climberPIDController1.setP(Constants.ClimberConstants.CLIMBER_P);
        climberPIDController1.setI(Constants.ClimberConstants.CLIMBER_I);
        climberPIDController1.setD(Constants.ClimberConstants.CLIMBER_D);
        climberPIDController1.setIZone(Constants.ClimberConstants.CLIMBER_IZONE);
        climberPIDController1.setFF(Constants.ClimberConstants.CLIMBER_FF);
        climberPIDController1.setOutputRange(Constants.ClimberConstants.CLIMBER_MIN, Constants.ClimberConstants.CLIMBER_MAX);

        climberPIDController2.setP(Constants.ClimberConstants.CLIMBER_P);
        climberPIDController2.setI(Constants.ClimberConstants.CLIMBER_I);
        climberPIDController2.setD(Constants.ClimberConstants.CLIMBER_D);
        climberPIDController2.setIZone(Constants.ClimberConstants.CLIMBER_IZONE);
        climberPIDController2.setFF(Constants.ClimberConstants.CLIMBER_FF);
        climberPIDController2.setOutputRange(Constants.ClimberConstants.CLIMBER_MIN, Constants.ClimberConstants.CLIMBER_MAX);

        climberMotor1.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 20);
        climberMotor1.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 40);
        climberMotor1.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 40);
        climberMotor1.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 100);
        climberMotor1.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 20);
        climberMotor1.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 400);
        climberMotor1.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 400);

        climberMotor2.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 20);
        climberMotor2.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 40);
        climberMotor2.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 40);
        climberMotor2.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 100);
        climberMotor2.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 20);
        climberMotor2.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 400);
        climberMotor2.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 400);


    }

    public toggleClimber(){
        if(isRaised){
            lowerClimber();
        } else {
            raiseClimber();
        }
    }

    public void moveClimber(Direction direction) {
        if (direction == Direction.UP) {
            raiseClimber();
        } else if (direction == Direction.DOWN) {
            lowerClimber();
        }
    }

    public void lowerClimber(){
        climberPIDController1.setReference(-Constants.ClimberConstants.ROTATION_DISTANCE, CANSparkMax.ControlType.kPosition);
        climberPIDController2.setReference(Constants.ClimberConstants.ROTATION_DISTANCE, CANSparkMax.ControlType.kPosition);
        isRaised = false;
    }

    public void raiseClimber(){
        climberPIDController1.setReference(0, CANSparkMax.ControlType.kPosition);
        climberPIDController2.setReference(0, CANSparkMax.ControlType.kPosition);
        isRaised = true;
    }

    public boolean isRaised(){
        return isRaised;
    }

}
