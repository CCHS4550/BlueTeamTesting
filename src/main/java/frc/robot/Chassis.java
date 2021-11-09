package frc.robot;

import frc.parent.*;

import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

/**
 * This class contains the code for the chassis
 * Any mechanism that would be considered the "Chassis" is included
 * In FRC 2020, this includes the DriveTrain aswell as code for shifting gbs
 * The gyro is also included
 */
public class Chassis implements RobotMap{

    public static AHRS gyro = new AHRS(SPI.Port.kMXP);

    public static CCSparkMax FRMax = new CCSparkMax(RobotMap.FORWARD_RIGHT, MotorType.kBrushless, IdleMode.kCoast, RobotMap.FR_REVERSE);
    public static CCSparkMax BRMax = new CCSparkMax(RobotMap.BACK_RIGHT, MotorType.kBrushless, IdleMode.kCoast, RobotMap.BR_REVERSE);
    public static CCSparkMax FLMax = new CCSparkMax(RobotMap.FORWARD_LEFT, MotorType.kBrushless, IdleMode.kCoast, RobotMap.FL_REVERSE);
    public static CCSparkMax BLMax = new CCSparkMax(RobotMap.BACK_LEFT, MotorType.kBrushless, IdleMode.kCoast, RobotMap.BL_REVERSE);
    
    public static Solenoid shiftSolOne = new Solenoid(RobotMap.SHIFT_SOLENOID_ONE);
    public static Solenoid shiftSolTwo = new Solenoid(RobotMap.SHIFT_SOLENOID_TWO);

    public static double leftSpd = 0;
    public static double rightSpd = 0; 

    /**
     * Pushes the stored speed values to the motors
     */
    public static void drive(){
        FRMax.setSpd(rightSpd);
        BRMax.setSpd(rightSpd);
        FLMax.setSpd(leftSpd);
        BLMax.setSpd(leftSpd);
    }

    /**
     * Sets the speed of the drive train
     * @param lspd The speed of the left side of the chassis
     * @param rspd The speed of the right side of the chassis
     */
    public static void driveSpd(double lspd, double rspd)
    {
        leftSpd = lspd;
        rightSpd = rspd; 
    }

    /**
     * Sets the stored speed values to controller inputs
     * @param yAxis The controller Y Axis
     * @param xAxis The controller X Axis
     */
    public static void driveAxis(double yAxis, double xAxis){
        leftSpd = OI.normalize((yAxis + xAxis), -1.0, 1.0);
        rightSpd = OI.normalize((yAxis - xAxis), -1.0, 1.0);
    }

    /**
     * Sets the PID Values for all motors
     * @param Kp
     * @param Ki
     * @param Kd
     * @param Ff
     */
    public static void setPID(double Kp, double Ki, double Kd, double Ff){
        FRMax.setPID(Kp, Ki, Kd, Ff);
        BRMax.setPID(Kp, Ki, Kd, Ff);
        FLMax.setPID(Kp, Ki, Kd, Ff);
        BLMax.setPID(Kp, Ki, Kd, Ff);
    }

    /**
     * Sets the target motor position
     * @param dist The distance in inches
     */
    public static void setTargetDistance(double dist, double maxSpd){
        FRMax.setMaxSpd(-maxSpd, maxSpd);
        BRMax.setMaxSpd(-maxSpd, maxSpd);
        FLMax.setMaxSpd(-maxSpd, maxSpd);
        BLMax.setMaxSpd(-maxSpd, maxSpd);
        BLMax.setTarget(-dist/RobotMap.WHEEL_CIRC, ControlType.kPosition);
        FRMax.setTarget(-dist/RobotMap.WHEEL_CIRC, ControlType.kPosition);
        BRMax.setTarget(-dist/RobotMap.WHEEL_CIRC, ControlType.kPosition);
        FLMax.setTarget(-dist/RobotMap.WHEEL_CIRC, ControlType.kPosition);
    }

    /**
     * Sets the Position Conversion factor of all drive motors
     * @param factor The Position Conversion Factor
     */
    public static void setPositionConversionFactor(double factor){
        FRMax.setPositionConversionFactor(factor);
        BRMax.setPositionConversionFactor(factor);
        FLMax.setPositionConversionFactor(factor);
        BLMax.setPositionConversionFactor(factor);
    }

    /**
     * Shifts the gearboxes
     * @param enable Whether the gearboxes should be in fast mode
     */
    public static void setFastMode(boolean fastMode)
    {
        if(fastMode)
        {
            FRMax.setPositionConversionFactor(0.1090909090909090);
            BRMax.setPositionConversionFactor(0.1090909090909090);
            FLMax.setPositionConversionFactor(0.1090909090909090);
            BLMax.setPositionConversionFactor(0.1090909090909090);
        }
        else
        {
            FRMax.setPositionConversionFactor(0.048);
            BRMax.setPositionConversionFactor(0.048);
            FLMax.setPositionConversionFactor(0.048);
            BLMax.setPositionConversionFactor(0.048);
        }
        shiftSolOne.set(!fastMode);
        shiftSolTwo.set(fastMode);

        FRMax.setPosition(0);
        BRMax.setPosition(0);
        FLMax.setPosition(0);
        BLMax.setPosition(0);
        
    }

    /**
     * Gets the angle of the gyro
     */
    public static double getAngle()
    {
        return gyro.getAngle();
    }

    /**
     * Gets the average position of all wheel encoders
     */
    public static double getPos()
    {
        return -((FRMax.getPosition() + BRMax.getPosition() + FLMax.getPosition() + BLMax.getPosition())/4) * RobotMap.WHEEL_CIRC;
        // return -FRMax.getPosition() * WHEEL_CIRC;
    }

    /**
     * Resets the positions of all motors and the gyro
     */
    public static void resetAll()
    {
        gyro.reset();
        FRMax.setPosition(0);
        BRMax.setPosition(0);
        FLMax.setPosition(0);
        BLMax.setPosition(0);

        FRMax.setMaxSpd(-1.0, 1.0);
        BRMax.setMaxSpd(-1.0, 1.0);
        FLMax.setMaxSpd(-1.0, 1.0);
        BLMax.setMaxSpd(-1.0, 1.0);
    }

    /**
     * Disables all chassis motors
     */
    public static void disableAll()
    {
        FRMax.disable();
        BRMax.disable();
        FLMax.disable();
        BLMax.disable();
    }
    /**
     * Rotates a given amount of degrees
     * @param degrees Target amount in degrees
     * @param proportion Proportion
     * @param rangeOfError Number of degrees + or - from the target to stop
     */
    //double proportion, double rangeOfError, double maxSpd
    public static void turnAngle(double degrees){
        double proportion = 0.05;
        double rangeOfError = 2;
        // double maxSpd = Chassis.DRIVE_SPD;
        double maxSpd = 1;
        // final double proportion = 0.5;
        // final double rangeOfError = 0.5;


        Timer t = new Timer();

        double angleDifference = degrees - getAngle();

        resetAll();

        FRMax.setSpd(0);
        BRMax.setSpd(0);
        FLMax.setSpd(0);
        BLMax.setSpd(0);
        
    

        t.reset();
        t.start();
        
        //(angleDifference > rangeOfError || angleDifference < -rangeOfError) && 
        // while(t.get() < Math.abs(degrees / 9)){
        while(t.get() < Math.abs(degrees / 18)){
            //Make negative
            angleDifference =   degrees - getAngle();
            FRMax.setSpd(-maxSpd * proportion * angleDifference * 2);
            BRMax.setSpd(-maxSpd * proportion * angleDifference * 2);
            FLMax.setSpd(maxSpd * proportion * angleDifference * 0.15);
            BLMax.setSpd(maxSpd * proportion * angleDifference * 0.15);
            // System.out.println("angleDifference: "+angleDifference);
            // System.out.println("Time Get:"+t.get());
        }

        t.stop();

        // FRMax.setSpd(0);
        // BRMax.setSpd(0);
        // FLMax.setSpd(0);
        // BLMax.setSpd(0);

        // gyro.reset();
        // gyro.resetDisplacement();
        // gyro.zeroYaw();

        // System.out.println("Rotated " + degrees +" degrees.");
        // System.out.println("Gyro angle:" +getAngle());
        // FRMax.setTarget(tar, controlType);
        // if(getAngle() - prevAngle == degrees){

        // }
        

        

        
        // BLMax.setTarget(-degrees , ControlType.kDutyCycle);
        // FRMax.setTarget(degrees, ControlType.kDutyCycle);
        // BRMax.setTarget(degrees, ControlType.kDutyCycle);
        // FLMax.setTarget(-degrees, ControlType.kDutyCycle);
    }
    
    public static void turnToAngle(float degree, double maxSpd){
        AHRS ahrs;
        
        
        PIDController turnController;
        double rotateToAngleRate;
        
        
        /* The following PID Controller coefficients will need to be tuned */
        /* to match the dynamics of your drive system.  Note that the      */
        /* SmartDashboard in Test mode has support for helping you tune    */
        /* controllers by displaying a form where you can enter new P, I,  */
        /* and D constants and test the mechanism.                         */
        
        final double kP = 0.03;
        final double kI = 0.00;
        final double kD = 0.00;
        final double kF = 0.00;
        //Work from here https://pdocs.kauailabs.com/navx-mxp/examples/rotate-to-angle-2/
        // turnController = new PIDController(kP, kI, kD, kF, ahrs);
      /* This tuning parameter indicates how close to "on target" the    */
      /* PID Controller will attempt to get.                             */
      
        final double kToleranceDegrees = 2.0f;

        // turnController.setSetpoint(degree);
    }


    
}