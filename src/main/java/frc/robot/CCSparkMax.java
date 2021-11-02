package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
// import com.revrobotics.CANEncoder;

//Documention: http://www.revrobotics.com/content/sw/max/sw-docs/java/com/revrobotics/CANSparkMax.html#%3Cinit%3E(int,com.revrobotics.CANSparkMaxLowLevel.MotorType)

/**
 * Compacts all methods needed to drive a SparkMax and Neo into one class
 */
public class CCSparkMax extends CANSparkMax{

    CANPIDController pidController;
    CANEncoder encoder;

    /**
     * CCSparkMax allows us to easily control Spark Max motor controllers
     * Information on modes can be found in the Spark Max documentation
     * @param deviceID The CAN channel of the motor controller
     * @param controlMode Specify whether the motor controller is operating in Brushed or Brushless mode
     * @param idleMode Specify whether the motor controller is set to Coast or Brake mode
     * @param reverse Reverses the direction of the motor controller
     */
    public CCSparkMax(int deviceID, MotorType controlMode, IdleMode idleMode, boolean reverse){
        super(deviceID, controlMode);
        pidController = super.getPIDController();

        if(controlMode == MotorType.kBrushed)
        {
            encoder = super.getEncoder(EncoderType.kNoSensor, 0);
        } else {
            encoder = super.getEncoder(EncoderType.kHallSensor, 0);
        }
        if(super.setIdleMode(idleMode) != CANError.kOk){
            System.out.println("Spark Max Idle Mode Not Set");
        }
        super.setInverted(reverse);
    }

    /**
     * Sets the speed of the motor controller
     * @param speed The speed that will be set (-1.0 to 1.0)
     */
    public void setSpd(double speed){
        super.set(speed);
    }

    public void disable(){
        super.disable();
    }

    /**
     * Sets the Position Conversion Factor for the encoder
     * @param factor The ratio of encoder units to desired units (ie. units -> in)
     */
    public void setPositionConversionFactor(double factor){
        encoder.setPositionConversionFactor(factor);
    }

    /**
     * Sets the encoder position
     * @param pos The new encoder position
     */
    public void setPosition(double pos){
        encoder.setPosition(pos);
    }

    /**
     * Sets the position(in encoder tics) of the neo
     * @param ref Position in encoder tics
     */
    public void setTarget(double tar, ControlType controlType){
        pidController.setReference(tar, controlType);
    }

    /**
     * Sets the velocity of the neo
     * @param vel Veloctiy of the neo
     */
    public void setVelocity(double vel){
        pidController.setReference(vel, ControlType.kVelocity);

    }

    /**
     * Returns the position of the encoder.
     * By default the position is in encoder units, but will return a distance if the Position Conversion Factor has been set.
     */
    public double getPosition(){
        return encoder.getPosition();
    }

    /**
     * Sets the PID values, must be positive
     * @param Kp The proportional gain value
     * @param Ki The integral gain value
     * @param Kd The derivative gain value
     */
    public void setPID(double Kp, double Ki, double Kd, double Ff){
        pidController.setP(Kp);
        pidController.setI(Ki);
        pidController.setD(Kd);
        pidController.setFF(Ff);
    }

    /**
     * Caps the speed of the motors
     * @param minSpd The minimum speed
     * @param maxSpd The maximum speed
     */
    public void setMaxSpd(double minSpd, double maxSpd)
    {
        pidController.setOutputRange(minSpd, maxSpd);
    }
}