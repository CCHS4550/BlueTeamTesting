package frc.parent; 

/**
 * This interface is designed to provide acsess to the various ports on the Joysticks
 * This interface is not to be touched or modified
 * 
 * To call the port: i.e. PilotMap.X_AXIS
 */

public interface PilotMap {
    //Stick ports
    public static int X_AXIS = 0;
    public static int Y_AXIS = 1;
    public static int Z_AXIS = 2;

    //Button Ports 
    public static int TRIGGER = 1;
    public static int STICK_BACK = 2;
    public static int STICK_MID = 3;
    public static int STICK_LEFT = 4;
    public static int STICK_RIGHT = 5;
    public static int PAD_LEFT_FRONT = 6;
    public static int PAD_LEFT_MID = 7;
    public static int PAD_LEFT_BACK = 8;
    public static int PAD_RIGHT_BACK = 9;
    public static int PAD_RIGHT_MID = 10;
    public static int PAD_RIGHT_FRONT = 11;

}