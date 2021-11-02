package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import frc.parent.*;

/**
 * This class acts as the main controller for the intake
 * Because there is only one intake, this class is static.
 * the class should NOT have an instance of it
 */

public class Intake implements RobotMap
{
    public static CCSparkMax max = new CCSparkMax(RobotMap.INTAKE, MotorType.kBrushless, IdleMode.kBrake, RobotMap.INTAKE_REVERSE);
    public static Solenoid sol1 = new Solenoid(RobotMap.INTAKE_SOLENOID_ONE);
    public static Solenoid sol2 = new Solenoid(RobotMap.INTAKE_SOLENOID_TWO);

    /**
     * @param extended If true, pnumatics will extend. 
     * This method is to extend the intake in and out of the frame perimeter
     */
    public static void setExtended(boolean extended)
    {
        sol1.set(extended);
        sol2.set(!extended);
    }

    /**
     * @param spd Sets the spd of the intake wheels. Domain: [-1, 1]
     */
    public static void setSpd(double spd)
    {
        if(!sol1.get())
        {
            max.setSpd(spd);
        }
        else
        {
            max.setSpd(-spd);
        }
    }

    
}