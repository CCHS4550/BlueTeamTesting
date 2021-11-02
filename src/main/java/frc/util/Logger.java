package frc.util;

import java.util.ArrayList;
import java.io.BufferedWriter;
import java.io.FileWriter;
// import java.io.IOException;

public class Logger
{
    public static ArrayList<String> logs = new ArrayList<String>();
    public static boolean hasBeenWritten = false;

    /**
     * Erases all logs from the ArrayList
     */
    public static void eraseLogs()
    {
        if(!hasBeenWritten)
        {
            System.out.println("Logs were not written and are being erased!");
        }
        logs = new ArrayList<String>();
        hasBeenWritten = false;
    }

    /**
     * Adds a log entry to the logs Arraylist
     * @param str The log to add to the ArrayList
     * @return The number of logs in the ArrayList
     */
    public static int logString(String str)
    {
        logs.add(str);
        return logs.size();
    }

    /**
     * Writes the logs in the ArrayList to a file
     * @param filePath The path of the log file that will be written
     * @return The number of logs in the ArrayList
     */
    public static int outputToFile(String filePath)
    {
        System.out.println("Outputting Logs To File <" + filePath + ">");
        try
        {
            BufferedWriter writer = new BufferedWriter(new FileWriter(filePath));
            String outStr = "";
            for(int i = 0; i < logs.size(); i++)
            {
                outStr += logs.get(i) + "\n";
            }
            writer.write(outStr);
            writer.close();
            hasBeenWritten = true;
            System.out.println("Log File Written");
        }
        catch(Exception e)
        {
            System.out.println("Logger Error!!");
            e.printStackTrace();
        }
        return logs.size();
    }

    public static int outputToConsole()
    {
        System.out.println("Printing All Logs to Console:");
        for(int i = 0; i < logs.size(); i++)
        {
            System.out.println("Log #" + i + ": " + logs.get(i));
        }
        return logs.size();
    }
}