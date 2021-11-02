package frc.robot;

import frc.parent.*;

// import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;

public class Climber implements RobotMap
{
    public static CCSparkMax max1 = new CCSparkMax(RobotMap.CLIMBER_LEFT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.CLIMBER_LEFT_REVERSE);
    public static CCSparkMax max2 = new CCSparkMax(RobotMap.CLIMBER_RIGHT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.CLIMBER_RIGHT_REVERSE);

    public static Solenoid sol1 = new Solenoid(RobotMap.CLIMBER_SOL_ONE);
    public static Solenoid sol2 = new Solenoid(RobotMap.CLIMBER_SOL_TWO);
    
    /**
     * Sets the speed of the right climber motor
     * @param spd Speed of the motor
     */
    public static void setRightSpd(double spd)
    {
        max2.set(spd);
    }

    /**
     * Sets the speed of the right climber motor
     * @param spd Speed of the motor
     */
    public static void setLeftSpd(double spd)
    {
        max1.set(spd);
    }

    /**
     * @param extended Whether the cylnders should be extended or not
     */
    public static void setExtended(boolean extended)
    {
        sol1.set(extended);
        sol2.set(!extended);
    }
}