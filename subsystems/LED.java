package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.BaseRobot;


import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
    LED m_LED;
    m_LED = new LED(this, "blinkin");
    m_LED.defaultPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    m_LED.pattern(RevBlinkinLedDriver.BlinkinPattern.RED)
        .duration(5000).blink(1500, 1000);

    pattern: Color or pattern number. Set this first.
    duration: How long before stopping the pattern in milliseconds
    blink: Have it blink and specify on time and off time in milliseconds
    default: default pattern (defaults to WHITE)
*/

public class LED extends SubsystemBase {
    private BaseRobot m_baseRobot;
    private RevBlinkinLedDriver m_blinkin;
    private RevBlinkinLedDriver.BlinkinPattern m_pattern;
    private RevBlinkinLedDriver.BlinkinPattern m_defaultPattern;
    private ElapsedTime m_durationTimer = new ElapsedTime();
    private ElapsedTime m_blinkTimer = new ElapsedTime();
    private int m_onDuration;
    private int m_offDuration;
    private int m_duration = -1;

    public enum LEDStates {
        IDLE,
        WAIT_BLINK_ON,
        WAIT_BLINK_OFF,
    }
    private volatile LEDStates m_LEDState = LEDStates.IDLE;

    public LED(BaseRobot baseRobot, final String blinkinName) {
        m_baseRobot = baseRobot;
        m_blinkin = m_baseRobot.hardwareMap.get(RevBlinkinLedDriver.class, blinkinName);
        m_pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        m_defaultPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        pattern(m_pattern);
    }

    public LED pattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        m_pattern = pattern;
        m_blinkin.setPattern(m_pattern);
        m_duration = -1;
        m_LEDState = LEDStates.IDLE;
        return this;
    }

    public LED defaultPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        m_defaultPattern = pattern;
        return this;
    }

    public LED useDefault() {
        pattern(m_defaultPattern);
        return this;
    }

    public LED duration(int duration) {
        m_duration = duration;
        m_durationTimer.reset();
        return this;
    }

    public LED blink(int onDuration, int offDuration) {
        m_onDuration = onDuration;
        m_offDuration = offDuration;
        m_blinkTimer.reset();
        m_LEDState = LEDStates.WAIT_BLINK_OFF;
        return this;
    }

    @Override
    public void periodic() {
        if (m_LEDState != LEDStates.IDLE && m_duration > 0 && m_durationTimer.milliseconds() > m_duration) {
            m_blinkin.setPattern(m_defaultPattern);
            m_LEDState = LEDStates.IDLE;
            m_duration = -1;
        }

        switch (m_LEDState) {
            case IDLE:
                break;
            case WAIT_BLINK_OFF:
                handleWaitBlinkOff();
                break;
            case WAIT_BLINK_ON:
                handleWaitBlinkOn();
                break;
        }
    }

    private void handleWaitBlinkOff() {
        if (m_blinkTimer.milliseconds() > m_onDuration) {
            m_blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            m_blinkTimer.reset();
            m_LEDState = LEDStates.WAIT_BLINK_ON;
        }
    }

    private void handleWaitBlinkOn() {
        if (m_blinkTimer.milliseconds() > m_offDuration) {
            m_blinkin.setPattern(m_pattern);
            m_blinkTimer.reset();
            m_LEDState = LEDStates.WAIT_BLINK_OFF;
        }
    }
}

/*

    // Fixed Palette Pattern
    RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER
    RevBlinkinLedDriver.BlinkinPattern.CONFETTI
    RevBlinkinLedDriver.BlinkinPattern.SHOT_RED
    RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE
    RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE
    RevBlinkinLedDriver.BlinkinPattern.SINELON_RAINBOW_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.SINELON_PARTY_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.FIRE_MEDIUM
    RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE
    RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.TWINKLES_PARTY_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.TWINKLES_OCEAN_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.TWINKLES_LAVA_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.TWINKLES_FOREST_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE
    RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED
    RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_GRAY
    RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED
    RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE
    RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY
    RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED
    RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE
    RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE
    RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY
    RevBlinkinLedDriver.BlinkinPattern.BREATH_RED
    RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE
    RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY
    RevBlinkinLedDriver.BlinkinPattern.STROBE_RED
    RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE
    RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD
    RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE

    // CP1: Color 1 Pattern
    RevBlinkinLedDriver.BlinkinPattern.CP1_END_TO_END_BLEND_TO_BLACK
    RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER
    RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE
    RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW
    RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_MEDIUM
    RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST
    RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW
    RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_FAST
    RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT
    RevBlinkinLedDriver.BlinkinPattern.CP1_STROBE

    // CP2: Color 2 Pattern
    RevBlinkinLedDriver.BlinkinPattern.CP2_END_TO_END_BLEND_TO_BLACK
    RevBlinkinLedDriver.BlinkinPattern.CP2_LARSON_SCANNER
    RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE
    RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_SLOW
    RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_MEDIUM
    RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST
    RevBlinkinLedDriver.BlinkinPattern.CP2_BREATH_SLOW
    RevBlinkinLedDriver.BlinkinPattern.CP2_BREATH_FAST
    RevBlinkinLedDriver.BlinkinPattern.CP2_SHOT
    RevBlinkinLedDriver.BlinkinPattern.CP2_STROBE

    // CP1_2: Color 1 and 2 Pattern
    RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_1_ON_2
    RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_2_ON_1
    RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT
    RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE
    RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND_1_TO_2
    RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND
    RevBlinkinLedDriver.BlinkinPattern.CP1_2_NO_BLENDING
    RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES
    RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES
    RevBlinkinLedDriver.BlinkinPattern.CP1_2_SINELON

    // Solid color
    RevBlinkinLedDriver.BlinkinPattern.HOT_PINK
    RevBlinkinLedDriver.BlinkinPattern.DARK_RED
    RevBlinkinLedDriver.BlinkinPattern.RED
    RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE
    RevBlinkinLedDriver.BlinkinPattern.ORANGE
    RevBlinkinLedDriver.BlinkinPattern.GOLD
    RevBlinkinLedDriver.BlinkinPattern.YELLOW
    RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN
    RevBlinkinLedDriver.BlinkinPattern.LIME
    RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN
    RevBlinkinLedDriver.BlinkinPattern.GREEN
    RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN
    RevBlinkinLedDriver.BlinkinPattern.AQUA
    RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE
    RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE
    RevBlinkinLedDriver.BlinkinPattern.BLUE
    RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET
    RevBlinkinLedDriver.BlinkinPattern.VIOLET
    RevBlinkinLedDriver.BlinkinPattern.WHITE
    RevBlinkinLedDriver.BlinkinPattern.GRAY
    RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY
    RevBlinkinLedDriver.BlinkinPattern.BLACK

*/
