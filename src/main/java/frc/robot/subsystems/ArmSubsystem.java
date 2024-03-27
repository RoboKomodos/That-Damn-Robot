package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.utils.Direction;

public class ArmSubsystem extends SubsystemBase{

    // Naming conventions like a car (i.e. facing the same direction as the robot like how driver side of car is on the left)
    private final CANSparkMax armMotorL;
    private final CANSparkMax armMotorR;
    private final SparkPIDController armPIDControllerL;
    private final SparkPIDController armPIDControllerR;
    private final RelativeEncoder armEncoderL;
    private final RelativeEncoder armEncoderR;

    private int armPositionIndex = 0;
    private Double[] armPositions = {0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0};

    public  ArmSubsystem(){
        // Motor objects, PID controllers, and encoders
        armMotorL = new CANSparkMax(Constants.ArmConstants.CAN_ID_ARM_MOTOR_L, MotorType.kBrushless);
        armMotorR = new CANSparkMax(Constants.ArmConstants.CAN_ID_ARM_MOTOR_R, MotorType.kBrushless);

        armMotorR.setIdleMode(CANSparkBase.IdleMode.kBrake);
        armMotorL.setIdleMode(CANSparkBase.IdleMode.kBrake);

        armPIDControllerL = armMotorL.getPIDController();
        armPIDControllerR = armMotorR.getPIDController();

        armEncoderL = armMotorL.getEncoder();
        armEncoderR = armMotorR.getEncoder();



        // Set PID constants
        armPIDControllerL.setP(Constants.ArmConstants.ARM_P);
        armPIDControllerL.setI(Constants.ArmConstants.ARM_I);
        armPIDControllerL.setD(Constants.ArmConstants.ARM_D);
        armPIDControllerL.setIZone(Constants.ArmConstants.ARM_IZONE);
        armPIDControllerL.setFF(Constants.ArmConstants.ARM_FF);
        armPIDControllerL.setOutputRange(Constants.ArmConstants.ARM_MIN, Constants.ArmConstants.ARM_MAX);

        armPIDControllerR.setP(Constants.ArmConstants.ARM_P);
        armPIDControllerR.setI(Constants.ArmConstants.ARM_I);
        armPIDControllerR.setD(Constants.ArmConstants.ARM_D);
        armPIDControllerR.setIZone(Constants.ArmConstants.ARM_IZONE);
        armPIDControllerR.setFF(Constants.ArmConstants.ARM_FF);
        armPIDControllerR.setOutputRange(Constants.ArmConstants.ARM_MIN, Constants.ArmConstants.ARM_MAX);

        armMotorL.follow(armMotorR, true);
    }

    // Set arm position based on an angle
    public void moveArm(double angle){
        armPIDControllerL.setReference(angle/360.0 * Constants.ArmConstants.conversionFactor, CANSparkMax.ControlType.kPosition);
        armPIDControllerR.setReference(angle/360.0 * Constants.ArmConstants.conversionFactor, CANSparkMax.ControlType.kPosition);
    }

    // Move arm up or down
    public void moveArm(Direction direction) {
        if (direction == Direction.UP) {
            armMotorR.set(Constants.ArmConstants.ARM_SPEED);
        } else if (direction == Direction.DOWN) {
            armMotorR.set(-Constants.ArmConstants.ARM_SPEED);
        }
    }

    // Stop arm movement
    public void stopArm() {
        armMotorR.stopMotor();
    }
}