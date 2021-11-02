package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.parent.RobotMap;

/**
 * This class is used to turn user input into values the code can understand
 * To this class is pretty much perfect, and should not be touched
 */
public class OI implements RobotMap{
    private static Joystick[] controllers = new Joystick[RobotMap.NUM_JOYSTICKS];

    /**
     * To support more than one joystick, we use and array
     * Initializes the array of joysticks
     * THIS METHOD MUST BE CALLED IN robotInit()
     */
    public static void initJoysticks()
    {
        for(int i = 0; i < controllers.length; i++){
            controllers[i] = new Joystick(i);
        }
    }

    /**
     * Takes in a button
     * @param contNum The Joystick object in the array you want to call
     * @param button Which button(From robotMap, or PilotMap) you want to use
     * @return A boolean. True = pressed, False = not pressed
     */
    public static boolean button(int contNum, int button){
        return controllers[contNum].getRawButton(button);
    }

    /**
     * Returns the position of an axis controller
     * @param contNum The Joystick object in the array you want to call
     * @param axis Which axis(from RobotMap or PilotMap) you want to use
     * @return A double w/ a domain of [-1, 1]
     */
    public static double axis(int contNum, int axis){
        return controllers[contNum].getRawAxis(axis);
    }

  
    /**
     * Allows all inputs to be capped at certain values
     * @param value The value that needs to be capped
     * @param min The minimum value it can be
     * @param max The maximum value it can be
     * @return Returns the capped value
     */
    public static double normalize(double value, double min, double max){
        if(value > max)
            return max;
        else if (value < min)
            return min;
        else
            return value;
    }

    /**
     * Overloaded version of normalize(value, min, max)
     * Deadband is designed with controllers in mind
     * @param value The value that needs to be capped
     * @param min The minimum value it can be
     * @param max The maximum value it can be
     * @param zone if |value| < zone its zero
     * @return The capped/deadbanded value
     */
    public static double normalize(double value, double min, double max, double zone){
        if(value < zone && value > -zone)
            value = 0;

        return normalize(value, min, max);
    }


}