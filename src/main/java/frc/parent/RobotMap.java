package frc.parent;

/**
 * This interface is desgined to store the ports for the robot
 * This inludes, can bus, polarity, pcm ports.
 * It can store any values that you may need to change quickly
 */
public interface RobotMap {
    public static final boolean ENABLE_COMPRESSOR = true;

    public static final int NUM_JOYSTICKS = 3;

    // Wheel Talons
    public static final int FORWARD_LEFT = 1;
    public static final boolean FL_REVERSE = false;
    public static final int BACK_LEFT = 2;
    public static final boolean BL_REVERSE = false;
    public static final int FORWARD_RIGHT = 3;
    public static final boolean FR_REVERSE = true;
    public static final int BACK_RIGHT = 4;
    public static final boolean BR_REVERSE = true;

    public static final int SHIFT_SOLENOID_ONE = 4;
    public static final int SHIFT_SOLENOID_TWO = 5;

    public static final double DRIVE_SPD = 0.85;

    // Motor Ports for Intake
    public static final int INTAKE = 6;
    public static final boolean INTAKE_REVERSE = false;
    public static final double INTAKE_SPD = 0.6;
    public static final int INTAKE_SOLENOID_ONE = 2;
    public static final int INTAKE_SOLENOID_TWO = 3;


    public static final int CLIMBER_LEFT = 11;
    public static final boolean CLIMBER_LEFT_REVERSE = false;
    public static final int CLIMBER_RIGHT = 12;
    public static final boolean CLIMBER_RIGHT_REVERSE = true;
    public static final double CLIMBER_SPD = 1.0;
    public static final int CLIMBER_SOL_ONE = 0;
    public static final int CLIMBER_SOL_TWO = 1;

    public static final int LOADER = 7;
    public static final boolean LOADER_REVERSE = false;
    public static final double LOADER_FWD_SPEED = 0.7;
    public static final double LOADER_REV_SPEED = -0.12;

    public static final int SPINDEXER = 5;
    public static final boolean SPINDEXER_REVERSE = true;
    public static final double SPINDEXER_SPEED = 1.0;
    public static final double SPINDEXER_AUTO_SPEED = 0.60;

    // Motor ports for the yeeter
    public static final int SHOOTER_ONE = 9;
    public static final int SHOOTER_TWO = 10;
    public static final boolean SHOOTER_ONE_REVERSE = true;
    public static final boolean SHOOTER_TWO_REVERSE = false;

    public static final int TURRET = 8;
    public static final boolean TURRET_REVERSE = false;
    public static final double TURRET_SPD = 0.25;
    public static final double TURRET_TARGETING_THRESHOLD = 1.0;
    public static final double TURRET_RANGE = 90; 

    public static final double WHEEL_CIRC = 7.25 * Math.PI;

}