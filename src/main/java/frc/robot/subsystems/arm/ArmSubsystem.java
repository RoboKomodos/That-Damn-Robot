package frc.robot.subsystems.arm;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import edu.wpi.first.wpilibj.Encoder;

import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{

    // Naming conventions like a car (i.e. facing the same direction as the robot like how driver side of car is on the left)
    private final CANSparkMax armMotorL;
    private final CANSparkMax armMotorR;
    private final SparkPIDController armPIDControllerL;
    private final SparkPIDController armPIDControllerR;
    private final RelativeEncoder armEncoderL;
    private final RelativeEncoder armEncoderR;

//    private final Encoder armEncoder;
    private final DutyCycleEncoder encoder;
    private final DigitalInput boreEncoder;

//    private final PIDController armPID;
//    private final AbsoluteEncoder armABSEnc;



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

//        armEncoder = new Encoder(8, 9, false, Encoder.EncodingType.k2X);
//        SmartDashboard.putNumber("Arm Encoder", armEncoder.getDistance());

        boreEncoder = new DigitalInput(0);
        encoder = new DutyCycleEncoder(boreEncoder);



       /* armABSEnc = new AbsoluteEncoder() {
            @Override
            public double getPosition() {
                return encoder.getAbsolutePosition();
            }

            @Override
            public double getVelocity() {
                return 0;
            }

            @Override
            public REVLibError setPositionConversionFactor(double factor) {
                return null;
            }

            @Override
            public double getPositionConversionFactor() {
                return 0;
            }

            @Override
            public REVLibError setVelocityConversionFactor(double factor) {
                return null;
            }

            @Override
            public double getVelocityConversionFactor() {
                return 0;
            }

            @Override
            public REVLibError setInverted(boolean inverted) {
                return null;
            }

            @Override
            public boolean getInverted() {
                return false;
            }

            @Override
            public REVLibError setAverageDepth(int depth) {
                return null;
            }

            @Override
            public int getAverageDepth() {
                return 0;
            }

            @Override
            public REVLibError setZeroOffset(double offset) {
                return null;
            }

            @Override
            public double getZeroOffset() {
                return 0;
            }
        };
        armPIDControllerR.setFeedbackDevice(armABSEnc);*/

//        armPID = new PIDController(Constants.ArmConstants.ARM_P, Constants.ArmConstants.ARM_I, Constants.ArmConstants.ARM_D);



//        armPIDControllerR.setFeedbackDevice();


//        encoder.setDistancePerRotation(4.0);
//        SmartDashboard.putNumber("Arm angle", encoder.getDistance());
        SmartDashboard.putBoolean("Is Encoder Connected", encoder.isConnected());





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

    public void periodic(){
        SmartDashboard.putNumber("Arm angle", encoder.getAbsolutePosition());
    }

    public Command raiseArm(){
        return run(() -> {
            //armMotorL.set(-Constants.ArmConstants.ARM_SPEED);
            armMotorR.set(Constants.ArmConstants.ARM_SPEED);
        });
    }

    public Command lowerArm(){
        return run(() -> {
            //armMotorL.set(Constants.ArmConstants.ARM_SPEED);
            armMotorR.set(-Constants.ArmConstants.ARM_SPEED);
        });
    }

    public Command stopArm(){
        return run(() -> {
//            armMotorL.set(0);
//            armMotorR.set(0);
            //armMotorL.stopMotor();
            armMotorR.stopMotor();
        });
    }

//    public Command setPosition(double angle){
//        return runOnce(() -> {
//            armPIDControllerL.setReference(angle/360.0 * Constants.ArmConstants.conversionFactor, CANSparkMax.ControlType.kPosition);
//            armPIDControllerR.setReference(angle/360.0 * Constants.ArmConstants.conversionFactor, CANSparkMax.ControlType.kPosition);
//        });
//    }

    public Command setArmPosition_STEPUP(){
        armPositionIndex++;
        return runOnce(() -> {
            setPosition(armPositions[armPositionIndex]);
        });
    }

    public Command setArmPosition_STEPDOWN(){
        armPositionIndex--;
        return runOnce(() -> {
            setPosition(armPositions[armPositionIndex]);
        });
    }

    public Command setArmPosition(double angle){
        return runOnce(() -> {
            setPosition(angle);
        });
    }

    private void setPosition(double angle){
        armPIDControllerL.setReference(angle/360.0 * Constants.ArmConstants.conversionFactor, CANSparkMax.ControlType.kPosition);
        armPIDControllerR.setReference(angle/360.0 * Constants.ArmConstants.conversionFactor, CANSparkMax.ControlType.kPosition);

//        armMotorR.set(armPID.calculate(encoder.getAbsolutePosition(), angle));
//        armMotorR.set(-armPID.calculate(encoder.getAbsolutePosition(), angle));
    }
}
