package frc.robot;

// import edu.wpi.first.wpilibj.Timer;

/**
 * This allows for the ability to have button presses, not just button holds
 */
public class ReleasableButton {
    boolean buttonState;
    int ticks;

    ReleasableButton()
    {
        buttonState = false;
        ticks = 0;
    }

    /**
     * @param isPressed Whether the Button is Pressed
     * @param delay The number of ticks before the button can be pressed again.
     * This allows for the button to be restricted to a certain number of pressed
     * Delay is how long until a driver can press a button
     */
    void updateButton(boolean isPressed, double delay)
    {
        if(ticks > delay)
        {
            if(isPressed && !buttonState)
            {
                buttonState = true;
                ticks = 0;
            } 
            else if(isPressed && buttonState)
            {
                buttonState = false;
                ticks = 0;
            }
        }
    }

    void tick()
    {
        ticks++;
    }

    /**
     * Increases the tick by one
     * @return The status of the button
     */
    boolean getStatus()
    {
        return buttonState;
    }

}
