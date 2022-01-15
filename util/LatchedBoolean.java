package org.firstinspires.ftc.teamcode.util;


public class LatchedBoolean {
    
    private boolean m_Last = false;
    
    public boolean update(boolean newValue) {
        boolean ret = false;
        if (newValue && !m_Last) {
            ret = true;
        }
        m_Last = newValue;
        return ret;
    }

    // todo: write your code here
}
