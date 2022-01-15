/*
Copyright 2021 FIRST Tech Challenge Team 13917

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.BaseRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
 
// control hub analog port 0 = "pixydigital", port 1 = "pixyanalog"
public class SUB_PixyCam extends SubsystemBase {
    private BaseRobot m_BaseRobot;

    private boolean debug = false;
    
    private AnalogInput m_PixyDigital; // Pixy2 digital pin 1 - hook to analog input for wiring convenience, used to indicate if an object has been detected
    private AnalogInput m_PixyAnalog; // pixy2 anaglog pin 8 

    private final int m_queueSize = 20;
    private int m_headQueueIndex = 0;
    private int[] m_voteQueue = new int[m_queueSize];
    int[] m_zoneCounts = new int[3];


    
    public SUB_PixyCam (BaseRobot p_BaseRobot, final String pixyDigital, final String pixyAnalog) {
        m_BaseRobot = p_BaseRobot;
        m_PixyDigital = m_BaseRobot.hardwareMap.analogInput.get(pixyDigital);
        m_PixyAnalog = m_BaseRobot.hardwareMap.analogInput.get(pixyAnalog);

        // Tracks how many votes for each zone have been seen.
        m_zoneCounts[0] = 0;
        m_zoneCounts[1] = 0;
        m_zoneCounts[2] = 0;

        for (int i = 0; i < m_queueSize; i++)
            m_voteQueue[i] = -1;
    }

    public boolean objectDetected() {
        return (m_PixyDigital.getVoltage()>2);
    }

    public double getVolt() {
        return m_PixyAnalog.getVoltage();
    }

public int getDuckPosition() {
    int DuckPosition;
    // determines where the robot should turn
        if ( getVolt() > 1.5){
            DuckPosition = -1;
        }
        else if (getVolt() < 1.5){
            DuckPosition = 1;
        }
        else {
            DuckPosition = 0;
        }
 return DuckPosition;
}

    public int getLevel() {
        int Level = 0; // 
        // determines which level our elevator should go to 
            if ( getVolt() > 2.1){
            Level = 1;    
            } 
            else if ( getVolt() <=2.1 && getVolt() >= 1.0  ){
            Level = 2;
            }
            else {
            Level = 3;    
            } 
        return Level;
    }
    
    
    // Remembers the last m_queueSize readings from the pixy cam.  Tracks how many
    // times each ring count was seen over that period.  Returns the most popular
    // ring count so far.
    public int getVotedLevelCount() {
        // Remove the oldest zone vote from the count totals.
        if (m_voteQueue[m_headQueueIndex] != -1)
            m_zoneCounts[m_voteQueue[m_headQueueIndex] - 1]--;
            
        int Level = getLevel();
        
        // Add the newest zone vote to the queue.
        if (Level == 1)
            m_voteQueue[m_headQueueIndex] = 1;
        else if (Level == 2)
            m_voteQueue[m_headQueueIndex] = 2;
        else if (Level == 3)
            m_voteQueue[m_headQueueIndex] = 3;

        // Add the newest ring count vote to the totals.
        m_zoneCounts[m_voteQueue[m_headQueueIndex] - 1]++;

        // Move to the next slot in the queue and wrap around to the beginning if necessary.
        m_headQueueIndex++;
        if (m_headQueueIndex == m_queueSize)
            m_headQueueIndex = 0;

        // Return the most popular ring count so far.
        if (m_zoneCounts[0] > m_zoneCounts[1] && m_zoneCounts[0] > m_zoneCounts[2])
            return 1;
        else if (m_zoneCounts[1] > m_zoneCounts[2])
            return 2;
        else
            return 3;
    }

    public void telemetry(){
        m_BaseRobot.telemetry.addData("Pixy: ", "Detected: %b, Volt=%.2f, Level=%d, voted=%d",
                            objectDetected(), getVolt(), getLevel(), getVotedLevelCount());
    }
    

    @Override
    public void periodic() {
        // telemetry();        
    }


}
