/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.revrobotics.*;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Class to control the RevRobotics Spark Max Motor Controller
 */
public class RevSparkMaxMotorController {
    public enum EncoderKind {
        INTERNAL_MAG, //Uses the Magnetic encoder that is built in to the REV SPARK MAX Brushless Motor
        EXTERNAL_MAG,
        QUAD,
        ANALOG_RELATIVE,
        ANALOG_ABSOLUTE,
    }

    private CANSparkMax motorController;

    //Limit switches will be normally open by default
    private SparkMaxLimitSwitch.Type forwardLimitSwitchNormallyOpen = SparkMaxLimitSwitch.Type.kNormallyOpen;
    private SparkMaxLimitSwitch.Type reverseLimitSwitchNormallyOpen = SparkMaxLimitSwitch.Type.kNormallyOpen;
    private static final int internalEncoderCountsPerRevolution = 42;
    private int encoderCountsPerRevolution = internalEncoderCountsPerRevolution;

    /**
     * Initialize the motor controller
     * @param motorId The ID of the motor in the CAN bus
     * @param isBrushless indicates if the motor is a Brushless or Brushed motor
     */
    public RevSparkMaxMotorController(int motorId, boolean isBrushless) {
        MotorType motorType;
        if (isBrushless) {
            motorType = MotorType.kBrushless;
        }else {
            motorType = MotorType.kBrushed;
        }

        motorController = new CANSparkMax(motorId, motorType);
    }

    /**
     * Tells the motor controller to use the specified type of Encoder to count revolutions of the shaft
     * @param encoderType The kind of the encoder to use
     * @param countsPerRevolution The number of ticks in one rotation of the motor shaft (NOTE: If encoderType is EncoderKind.INTERNAL_MAG then this will parameter will be ignored)
     */
    public void configureEncoder(EncoderKind encoderType, int countsPerRevolution) {
        SparkMaxPIDController pidController = motorController.getPIDController();
        encoderCountsPerRevolution = countsPerRevolution;

        if (encoderType == EncoderKind.ANALOG_RELATIVE) {
            /*CANAnalog analogEncoder = new CANAnalog(motorController, AnalogMode.kRelative);
            // Line above changed to the line below 2/4/2022 by Dallin Wright because CANAnalog class was depreciated */
            SparkMaxAnalogSensor analogEncoder = motorController.getAnalog(SparkMaxAnalogSensor.Mode.kRelative); // WAS CANAnalog, changed 2/4/2022 by Dallin Wright because class was depreciated
            pidController.setFeedbackDevice(analogEncoder);
        }else if (encoderType == EncoderKind.ANALOG_ABSOLUTE) {
            /*CANAnalog analogEncoder = new CANAnalog(motorController, AnalogMode.kAbsolute);
            // Line above changed to the line below 2/4/2022 by Dallin Wright because CANAnalog class was depreciated */
            SparkMaxAnalogSensor analogEncoder = motorController.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute); // WAS CANAnalog, changed 2/4/2022 by Dallin Wright because class was depreciated
            pidController.setFeedbackDevice(analogEncoder);
        }else if (encoderType == EncoderKind.INTERNAL_MAG) {
            /*CANEncoder magEncoder = new CANEncoder(motorController, EncoderType.kHallSensor, internalEncoderCountsPerRevolution);
            encoderCountsPerRevolution = internalEncoderCountsPerRevolution;
            // Line above changed to the line below 2/4/2022 by Dallin Wright because CANEncoder class was depreciated*/
            RelativeEncoder magEncoder = motorController.getEncoder();
            pidController.setFeedbackDevice(magEncoder);
        }else if (encoderType == EncoderKind.QUAD) {
            /*CANEncoder quadEncoder = new CANEncoder(motorController, EncoderType.kQuadrature, countsPerRevolution);
            // Line above changed to the line below 2/4/2022 by Dallin Wright because CANEncoder class was depreciated*/
            RelativeEncoder quadEncoder = motorController.getEncoder();
            pidController.setFeedbackDevice(quadEncoder);
        }else if (encoderType == EncoderKind.EXTERNAL_MAG) {
            /*CANEncoder magEncoder = new CANEncoder(motorController, EncoderType.kHallSensor, countsPerRevolution);
            // Line above changed to the line below 2/4/2022 by Dallin Wright because CANEncoder class was depreciated*/
            RelativeEncoder magEncoder = motorController.getEncoder();
            pidController.setFeedbackDevice(magEncoder);
        }
    }

    /**
     * Gets the base class the RevSparkMaxMotorController class is built on
     */
    public CANSparkMax getRealMotorController() {
        return motorController;
    }

    /**
     * Makes this motor controller mirror the output of the given motor controller
     * @param leader The motor controller to mirror
     * @param invertOuput Indicates if the output should be the opposite of the leader
     */
    public void follow(RevSparkMaxMotorController leader, boolean invertOutput) {
        motorController.follow(leader.getRealMotorController(), invertOutput);
        
    }

    /**
     * Sets the speed of the motor as a percentage of its full power (voltage)
     * @param power The speed to set the motor to as a percentage from -1.0 to +1.0
     */
    public void setPercentSpeed(double power) {
        motorController.set(power);
    }

    /**
     * Returns the current power output of the motor as a percentage between -1 and 1
     * @return Power output from -1 to 1
     */
    public double getPercentSpeed () {
        return motorController.get();
    }

    /**
     * Gets the current voltage that the motor controller is outputting to the motor
     */
    public double getVoltage() {
        return motorController.getBusVoltage();
    }
    /**
     * Returns the number of counts the motor has turned
     * @return Number of 'counts' the shaft has rotated
     */
    public double getEncoderPosition () {
        RelativeEncoder encoder = motorController.getEncoder();
        
        return encoder.getPosition() * encoderCountsPerRevolution;
    }

    /**
     * How many times the shaft of the encoder has made a full revolution
     */
    public double getEncoderRotations() {
        RelativeEncoder encoder = motorController.getEncoder();
        
        return encoder.getPosition();
    }

    /**
     * Inverts the direction the motor will spin
     * For example: If the motor is inverted, providing a positive speed will result in the motor moving backwards rather than forwards
     */
    public void invertMotor(boolean invert) {
        motorController.setInverted(invert);
    }

    /**
     * Change the brake mode
     * @param brakeEnabled If true, brakes will be enabled
     */
    public void setBrakeEnabled (boolean brakeEnabled) {
        IdleMode idleMode;
        
        if (brakeEnabled) {
            idleMode = IdleMode.kBrake;
        }
        else {
            idleMode = IdleMode.kCoast;
        }

        motorController.setIdleMode(idleMode);
    }

    public boolean getBrakeEnabled() {
        return motorController.getIdleMode() == IdleMode.kBrake;
    }

    public void setIMaxAccumulation(double maxAcc) {
        SparkMaxPIDController controller = motorController.getPIDController();
        controller.setIMaxAccum(maxAcc, 0);
    }

    

    /**
     * Sets the speed in RPM (Rotations per Minute)
     * @param speedRPM The speed in RPM
     */
    public void setVelocityRPM (double speedRPM) {
        double adjustedRPM = speedRPM / 0.6; //this is a fudge factor
        SparkMaxPIDController pidController = motorController.getPIDController();
        pidController.setReference(adjustedRPM, CANSparkMax.ControlType.kVelocity, 0);
    }

    /**
     * Sets the motor speed in RPS (Rotations per second)
     * @param speedRPS The speed in RPS
     */
    public void setVelocityRPS(double speedRPS) {
        double speedRPStoRPM = speedRPS * 60;//Convert RPS to RPM because the motor controller wants speed in RPM
        SparkMaxPIDController pidController = motorController.getPIDController();
        pidController.setReference(speedRPStoRPM,CANSparkMax.ControlType.kVelocity, 0);
    }

    /**
     * Gets the current velocity of the motor in RPS (Rotations per Second)
     */
    public double getVelocityRPS() {
        double rpm = getVelocityRPM();

        return rpm/60;//Convert RPM to RPS
    }

    public double getVelocityRPM() {
        RelativeEncoder encoder = motorController.getEncoder();
        return encoder.getVelocity();
    }

    public void configurePIDController (double p, double i, double d, double allowedError) {
        SparkMaxPIDController pidController = motorController.getPIDController();
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        pidController.setSmartMotionAllowedClosedLoopError(allowedError, 0);
    }
    
    /**
     * Enables forward limit switch so that when triggered it will stop the motor from turning forward
     * @param enable A boolean indicating whether to enable or disable the forward limit switch
     * @param normallyOpen A boolean indicating if the limit switch is normally open or not (is the switch triggered when nothing is pushing on it)
     */
    public void enableForwardLimitSwitch(boolean enable, boolean normallyOpen) {
        //Set if the switch is normally open or closed
        if (normallyOpen) {
            forwardLimitSwitchNormallyOpen = SparkMaxLimitSwitch.Type.kNormallyOpen;
        } else {
            forwardLimitSwitchNormallyOpen = SparkMaxLimitSwitch.Type.kNormallyClosed;
        }

        motorController.enableSoftLimit(SoftLimitDirection.kForward, enable);
    }

    /**
     * Enables reverse limit switch so that when triggered it will stop the motor from turning backward
     * @param enable A boolean indicating whether to enable or disable the forward limit switch
     * @param normallyOpen A boolean indicating if the limit switch is normally open or not (is the switch triggered when nothing is pushing on it)
     */
    public void enableReverseLimitSwitch(boolean enable, boolean normallyOpen) {
        if (normallyOpen) {
            reverseLimitSwitchNormallyOpen = SparkMaxLimitSwitch.Type.kNormallyOpen;
        } else {
            reverseLimitSwitchNormallyOpen = SparkMaxLimitSwitch.Type.kNormallyClosed;
        }
        
        motorController.enableSoftLimit(SoftLimitDirection.kReverse, enable);
    }

    /**
     * Checks if the forward limit switch is triggered currently
     * NOTE: Ensure you call enableForwardLimitSwitch() method to ensure your switch is NormallyOpen or NormallyClosed
     * @return A boolean indicating if the limit switch is triggered
     */
    public boolean getForwardLimitSwitchTriggered() {
        SparkMaxLimitSwitch limitSwitch = motorController.getForwardLimitSwitch(forwardLimitSwitchNormallyOpen);
        
        return limitSwitch.isPressed();
    }

    /**
     * Checks if the reverse limit switch is triggered currently
     * NOTE: Ensure you call enableReverseLimitSwitch() method to ensure your switch is NormallyOpen or NormallyClosed
     * @return A boolean indicating if the limit switch is triggered
     */
    public boolean getReverseLimitSwitchTriggered() {
        SparkMaxLimitSwitch limitSwitch = motorController.getReverseLimitSwitch(reverseLimitSwitchNormallyOpen);

        return limitSwitch.isPressed();
    }

    /**
     * Gets the ID of the motor controller in the CAN bus
     * @return The ID of motor controller
     */
    public int getMotorControllerId() {
        return motorController.getDeviceId();
    }

    /**
     * How many amps the motor is currently putting out to make the shaft spin
     * @return The Current in Amps
     */
    public double getMotorAmperageOutput() {
        return motorController.getOutputCurrent();
    }

    /**
     * "Zeros" or "resets" the encoder so that the current encoder position is considered 0
     */
    public void zeroEncoder() {
        RelativeEncoder encoder = motorController.getEncoder();
        encoder.setPosition(0);
    }
}
