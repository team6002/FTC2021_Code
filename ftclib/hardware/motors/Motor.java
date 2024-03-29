package org.firstinspires.ftc.teamcode.ftclib.hardware.motors;

import org.firstinspires.ftc.teamcode.ftclib.controller.PController;
import org.firstinspires.ftc.teamcode.ftclib.controller.PIDController;
// import com.arcrobotics.ftclib.controller.PController;
// import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import org.firstinspires.ftc.teamcode.ftclib.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Supplier;

/**
 * This is the common wrapper for the {@link DcMotor} object in the
 * FTC SDK.
 *
 * @author Jackson
 */
public class Motor implements HardwareDevice {

    public enum GoBILDA {
        RPM_30(5264, 30), RPM_43(3892, 43), RPM_60(2786, 60), RPM_84(1993.6, 84),
        RPM_117(1425.2, 117), RPM_223(753.2, 223), RPM_312(537.6, 312), RPM_435(383.6, 435),
        RPM_1150(145.6, 1150), RPM_1620(103.6, 1620), BARE(28,6000), NONE(0, 0);

        private double cpr, rpm;

        GoBILDA(double cpr, double rpm) {
            this.cpr = cpr;
            this.rpm = rpm;
        }

        public double getCPR() {
            return cpr;
        }

        public double getRPM() {
            return rpm;
        }

        public double getAchievableMaxTicksPerSecond() {
            return cpr * rpm / 60;
        }
    }

    public enum Direction {
        FORWARD(1), REVERSE(-1);

        private int val;

        Direction(int multiplier) {
            val = multiplier;
        }

        public int getMultiplier() {
            return val;
        }
    }

    public class Encoder {

        private DcMotor m_motor;
        private int resetVal, lastPosition;
        private Direction direction;
        private double lastTimeStamp, veloEstimate, dpp;

        /**
         * The encoder object for the motor.
         *
         * @param position  the position supplier which just points to the
         *                  current position of the motor in ticks
         */
        public Encoder(DcMotor motor) {
            m_motor = motor;
            dpp = 1;
            resetVal = 0;
            lastPosition = 0;
            veloEstimate = 0;
            direction = Direction.FORWARD;
            lastTimeStamp = (double)System.nanoTime() / 1E9;
        }

        /**
         * @return  the current position of the encoder
         */
        public int getPosition() {
            int currentPosition = m_motor.getCurrentPosition();
            if (currentPosition != lastPosition) {
                double currentTime = (double)System.nanoTime() / 1E9;
                double dt = currentTime - lastTimeStamp;
                veloEstimate = (currentPosition - lastPosition) / dt;
                lastPosition = m_motor.getCurrentPosition();
                lastTimeStamp = currentTime;
            }
            return direction.getMultiplier() * (currentPosition - resetVal);
        }

        /**
         * @return  the distance traveled by the encoder
         */
        public double getDistance() {
            return dpp * getPosition();
        }

        /**
         * Resets the encoder without having to stop the motor.
         */
        public void reset() {
            // resetVal = getPosition();
            resetVal = m_motor.getCurrentPosition();
        }

        /**
         * Sets the distance per pulse of the encoder.
         *
         * @param distancePerPulse the desired distance per pulse (in units per tick)
         */
        public Encoder setDistancePerPulse(double distancePerPulse) {
            dpp = distancePerPulse;
            return this;
        }

        /**
         * Sets the direction of the encoder to forward or reverse
         *
         * @param direction  the desired direction
         */
        public void setDirection(Direction direction) {
            this.direction = direction;
        }

        /**
         * @return  the number of revolutions turned by the encoder
         */
        public double getRevolutions() {
            return getPosition() / getCPR();
        }

        public double getRawVelocity() {
            return getVelocity();
        }

        private final static int CPS_STEP = 0x10000;

        /**
         * Corrects for velocity overflow
         *
         * @return the corrected velocity
         */
        public double getCorrectedVelocity() {
            double real = getRawVelocity();
            while (Math.abs(veloEstimate - real) > CPS_STEP / 2.0) {
                real += Math.signum(veloEstimate - real) * CPS_STEP;
            }
            return real;
        }

    }

    /**
     * The RunMode of the motor.
     */
    public enum RunMode {
        VelocityControl, PositionControl, RawPower
    }

    public enum ZeroPowerBehavior {
        UNKNOWN(DcMotor.ZeroPowerBehavior.UNKNOWN),
        BRAKE(DcMotor.ZeroPowerBehavior.BRAKE),
        FLOAT(DcMotor.ZeroPowerBehavior.FLOAT);

        private final DcMotor.ZeroPowerBehavior m_behavior;

        ZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
            m_behavior = behavior;
        }

        public DcMotor.ZeroPowerBehavior getBehavior() {
            return m_behavior;
        }
    }

    public DcMotor motor;
    public Encoder encoder;

    /**
     * The runmode of the motor
     */
    protected RunMode runmode;

    /**
     * The achievable ticks per second velocity of the motor
     */
    public double ACHIEVABLE_MAX_TICKS_PER_SECOND;

    /**
     * The motor type
     */
    protected GoBILDA type;

    protected PIDController veloController = new PIDController(1,0,0);;
    protected PController positionController = new PController(1);

    protected SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 1, 0);

    private boolean targetIsSet = false;

    public Motor() {}

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap  the hardware map from the OpMode
     * @param id    the device id from the RC config
     */
    public Motor(HardwareMap hMap, String id) {
        motor = hMap.get(DcMotor.class, id);
        runmode = RunMode.RawPower;
        type = GoBILDA.NONE;
        ACHIEVABLE_MAX_TICKS_PER_SECOND = motor.getMotorType().getAchieveableMaxTicksPerSecond();
        encoder = new Encoder(motor);
    }

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap        the hardware map from the OpMode
     * @param id          the device id from the RC config
     * @param gobildaType the type of gobilda 5202 series motor being used
     */
    public Motor(HardwareMap hMap, String id, GoBILDA gobildaType) {
        motor = hMap.get(DcMotor.class, id);
        runmode = RunMode.RawPower;
        type = gobildaType;
        ACHIEVABLE_MAX_TICKS_PER_SECOND = gobildaType.getAchievableMaxTicksPerSecond();
        encoder = new Encoder(motor);
    }

    /**
     * Constructs an instance motor for the wrapper
     *
     * @param hMap      the hardware map from the OpMode
     * @param id        the device id from the RC config
     * @param cpr       the counts per revolution of the motor
     * @param rpm       the revolutions per minute of the motor
     */
    public Motor(HardwareMap hMap, String id, double cpr, double rpm) {
        motor = hMap.get(DcMotor.class, id);
        runmode = RunMode.RawPower;
        type = GoBILDA.NONE;
        ACHIEVABLE_MAX_TICKS_PER_SECOND = cpr * rpm / 60;
        encoder = new Encoder(motor);
    }

    /**
     * Common method for setting the speed of a motor.
     *
     * @param output The percentage of power to set. Value should be between -1.0 and 1.0.
     */
    public void set(double output) {
        if (runmode == RunMode.VelocityControl) {
            double speed = output * ACHIEVABLE_MAX_TICKS_PER_SECOND;
            double velocity = veloController.calculate(getVelocity(), speed) + feedforward.calculate(speed);
            motor.setPower(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND);
        } else if (runmode == RunMode.PositionControl) {
            double error = positionController.calculate(encoder.getPosition());
            motor.setPower(output * error);
        } else {
            motor.setPower(output);
        }
    }

    /**
     * Sets the distance per pulse of the encoder in units per tick.
     * @param distancePerPulse  the desired distance per pulse
     * @return an encoder an object with the specified distance per pulse
     */
    public Encoder setDistancePerPulse(double distancePerPulse) {
        return encoder.setDistancePerPulse(distancePerPulse);
    }

    /**
     * @return  the distance traveled by the encoder
     */
    public double getDistance() {
        return encoder.getDistance();
    }

    /**
     * @return if the motor is at the target position
     */
    public boolean atTargetPosition() {
        return positionController.atSetPoint();
    }

    /**
     * Resets the encoder.
     */
    public void resetEncoder() {
        encoder.reset();
    }

    /**
     * @return the velocity coefficients
     */
    public double[] getVeloCoefficients() {
        return veloController.getCoefficients();
    }

    /**
     * @return the position coefficient
     */
    public double getPositionCoefficient() {
        return positionController.getP();
    }

    /**
     * @return the feedforward coefficients
     */
    public double[] getFeedforwardCoefficients() {
        return new double[]{feedforward.ks, feedforward.kv};
    }

    /**
     * A wrapper method for the zero power behavior
     *
     * @param behavior the behavior desired
     */
    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior.getBehavior());
    }

    /**
     * @return  the current position of the motor in ticks
     */
    public int getCurrentPosition() {
        return encoder.getPosition();
    }

    /**
     * @return  the corrected velocity for overflow
     */
    public double getCorrectedVelocity() {
        return encoder.getCorrectedVelocity();
    }

    /**
     * @return the counts per revolution of the motor
     */
    public double getCPR() {
        return type == GoBILDA.NONE ? motor.getMotorType().getTicksPerRev() : type.getCPR();
    }

    /**
     * @return the max RPM of the motor
     */
    public double getMaxRPM() {
        return type == GoBILDA.NONE ? motor.getMotorType().getMaxRPM() : type.getRPM();
    }

    /**
     * Sets the {@link RunMode} of the motor
     *
     * @param runmode the desired runmode
     */
    public void setRunMode(RunMode runmode) {
        this.runmode = runmode;
        veloController.reset();
        positionController.reset();
        if (runmode == RunMode.PositionControl && !targetIsSet) {
            setTargetPosition(getCurrentPosition());
            targetIsSet = false;
        }
    }

    protected double getVelocity() {
        return get() * ACHIEVABLE_MAX_TICKS_PER_SECOND;
    }

    /**
     * Common method for getting the current set speed of a motor.
     *
     * @return The current set speed. Value is between -1.0 and 1.0.
     */
    public double get() {
        return motor.getPower();
    }

    /**
     * Sets the target position for the motor to the desired target.
     * Once {@link #set(double)} is called, the motor will attempt to move in the direction
     * of said target.
     *
     * @param target
     */
    public void setTargetPosition(int target) {
        targetIsSet = true;
        positionController.setSetPoint(target);
    }

    public void setPositionTolerance(double tolerance) {
        positionController.setTolerance(tolerance);
    }

    /**
     * Common method for inverting direction of a motor.
     *
     * @param isInverted The state of inversion true is inverted.
     */
    public void setInverted(boolean isInverted) {
        motor.setDirection(isInverted ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        encoder.setDirection(isInverted ? Direction.REVERSE : Direction.FORWARD);
    }

    /**
     * Common method for returning if a motor is in the inverted state or not.
     *
     * @return isInverted The state of the inversion true is inverted.
     */
    public boolean getInverted() {
        return DcMotor.Direction.REVERSE == motor.getDirection();
    }

    /**
     * Set the velocity pid coefficients for the motor.
     *
     * @param kp    the proportional gain
     * @param ki    the integral gain
     * @param kd    the derivative gain
     */
    public void setVeloCoefficients(double kp, double ki, double kd) {
        veloController.setPIDF(kp, ki, kd, 0);
    }

    /**
     * Set the feedforward coefficients for the motor.
     *
     * @param ks    the static gain
     * @param kv    the velocity gain
     */
    public void setFeedforwardCoefficients(double ks, double kv) {
        feedforward = new SimpleMotorFeedforward(ks, kv);
    }

    /**
     * Set the proportional gain for the position controller.
     *
     * @param kp    the proportional gain
     */
    public void setPositionCoefficient(double kp) {
        positionController.setP(kp);
    }

    /**
     * Disable the motor.
     */
    @Override
    public void disable() {
        motor.close();
    }

    @Override
    public String getDeviceType() {
        return "Motor " + motor.getDeviceName() + " from " + motor.getManufacturer()
                + " in port " + motor.getPortNumber();
    }

    /**
     * Stops motor movement. Motor can be moved again by calling set without having to re-enable the
     * motor.
     */
    public void stopMotor() {
        set(0);
    }

}
