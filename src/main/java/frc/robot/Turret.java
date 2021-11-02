package frc.robot;

import frc.parent.*;
import frc.robot.sensors.LemonLight;

import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;

// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.wpilibj.AnalogPotentiometer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.NetworkTableType;
// import edu.wpi.first.networktables.NetworkTableValue;

// import com.revrobotics.CANEncoder;

/**
 * Implements the shooter and turret mechanisms
 */
public class Turret implements RobotMap{
    // public AnalogPotentiometer potent = new AnalogPotentiometer(RobotMap.POTENTIOMETER);
    public static CCSparkMax maxTurret = new CCSparkMax(RobotMap.TURRET, MotorType.kBrushless, IdleMode.kBrake, RobotMap.TURRET_REVERSE);
    public static CCSparkMax maxShooter1 = new CCSparkMax(RobotMap.SHOOTER_ONE, MotorType.kBrushless, IdleMode.kBrake, RobotMap.SHOOTER_ONE_REVERSE);
    public static CCSparkMax maxShooter2 = new CCSparkMax(RobotMap.SHOOTER_TWO, MotorType.kBrushless, IdleMode.kBrake, RobotMap.SHOOTER_TWO_REVERSE);

    /**
     * Sets the spin of the turret
     * @param spd Domina: [-1, 1]
     */
    public static void setSpin(double spd){
        if(spd > 0 && !(getEncoder() > TURRET_RANGE))
            maxTurret.setSpd(spd);
        else if(spd < 0 && !(getEncoder() < -TURRET_RANGE))
            maxTurret.setSpd(spd);
        else 
            maxTurret.setSpd(0);
    }

    /**
     * Sets both flywheel speeds (voltage)
     * @param spd Domain: [-1, 1]
     */
    public static void setShooterRaw(double spd)
    {
        maxShooter1.setSpd(spd);
        maxShooter2.setSpd(spd);
        // System.out.println(maxShooter1.getOutputCurrent());
    }

    /**
     * Sets both flywheel speeds (Velocity)
     * @param spd Domain: [-1, 1]
     */
    public static void setShooterVelocity(double vel)
    {
        maxShooter1.setSpd(vel);
        maxShooter2.setSpd(vel);
        // System.out.println(maxShooter1.getOutputCurrent());
    }

    /** -+
     * Sets the PID consts for pidController
     * @param Kp P const
     * @param Ki I const
     * @param Kd D const
     * @param Ff Feed forward 
     */
    public static void setConst(double Kp, double Ki, double Kd, double Ff){
        maxTurret.setPID(Kp, Ki, Kd, Ff);
    }
    
    public static double getEncoder(){
        return maxTurret.getPosition();
    }

    public static void reset(){
        maxTurret.setPosition(0);
        Turret.maxTurret.setIdleMode(IdleMode.kBrake);
    }

    public static void restrictTurret(){
    }

    /**
     * Locks on to the target when LemonLight is active
     */
    public static void lockOn(){
        // System.out.println("Has Target: " + LemonLight.hasTarget());
        if(LemonLight.hasTarget())
        {
            // double outYaw = OI.normalize(LemonLight.getYaw(), -RobotMap.TURRET_SPD, RobotMap.TURRET_SPD, RobotMap.TURRET_TARGETING_THRESHOLD);

            // System.out.println("Out Yaw: " + outYaw);
            // setSpin(OI.normalize(outYaw, -0.15, 0.15));
            maxTurret.setPosition(0.0);
            maxTurret.setTarget(LemonLight.getYaw(), ControlType.kPosition);
        }
        else
        {
            // System.out.println("No Target");
            setSpin(0.0);
            maxTurret.setSpd(0.0);
           // maxShooter2.setVelocity(0.85);
        }
    }

    /**
     * Locks on to the target for a period of time
     * @param time Time to target for in seconds
     */
    public static void lockOnForTime(double time)
    {
        for(int i = 0; i < 500.0 * time; i++)
        {
          Turret.lockOn();
          Timer.delay(time/(500.0 * time));
        }
    }

    /**
     * Uses the tested equation to get the desired shooter speed from the LemonLight distacne
     * @return The correct shooter speed
     */
    public static double getShooterVelocityFromDistance()
    {
        double dist = LemonLight.distToTarget();
        return OI.normalize(0.0239 * Math.pow(dist, 2) - 0.1015 * dist + 0.9231, -1.0, 1.0);
    }

    /**
     * @return The total current from the shooter motors
     */
    public static double getShooterCurrent()
    {
        return maxShooter1.getOutputCurrent() + maxShooter2.getOutputCurrent();
    }

}
