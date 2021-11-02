package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.parent.*;

/**
 * The magazine ("Spindexer") of the robot and the loader wheel
 * Call it the revolver, and I will murder you
 */
public class Loader implements RobotMap
{
    public static CCSparkMax spinMax = new CCSparkMax(RobotMap.SPINDEXER, MotorType.kBrushless, IdleMode.kBrake, RobotMap.SPINDEXER_REVERSE);
    public static CCSparkMax loaderMax = new CCSparkMax(RobotMap.LOADER, MotorType.kBrushless, IdleMode.kBrake, RobotMap.LOADER_REVERSE);

    /**
     * Sets the spd of the SPINDEXER
     * @param spd Domain: [-1, 1]
     */
    public static void setSpinSpd(double spd)
    {
       spinMax.setSpd(spd);
    }

    /**
     * Sets the spd of the LOADER WHEEL
     * @param spd Domain: [-1, 1]
     */
    public static void setLoaderSpd(double spd)
    {
        loaderMax.setSpd(spd);
    }

    public static void reset()
    {
        spinMax.setPosition(0.0);
        spinMax.setPositionConversionFactor(1.0/462.2);
        loaderMax.setPosition(0.0);
    }

    public static double printSpin(){
        return spinMax.getOutputCurrent();
    }
}