package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;


public class RateLimiter {


        private ElapsedTime runtime = new ElapsedTime();
        private double m_Rate; // the change / second
        private boolean m_started=false;
    
    
        public RateLimiter() {
            
        }
        
        public double getMax(double maxValue,double rate) {
            if (!m_started) {
                runtime.reset();    
                m_started = true;
            }
            
            double returnValue = maxValue * (runtime.milliseconds()/1000) / rate;
            if (returnValue > maxValue) returnValue = maxValue;
            return returnValue;
        }
        
        
}
