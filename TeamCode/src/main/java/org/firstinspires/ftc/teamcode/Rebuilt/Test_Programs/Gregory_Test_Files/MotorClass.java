    package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files;

    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.HardwareMap;

    /**
     * Represents an FTC motor with optional PID velocity control.
     * Simple, single-class design similar to ServoClass.
     */
    class MotorClass {
        public enum MotorType {
            BASIC_MOTOR,
            EXTENDED_MOTOR
        }

        private String hardwareMapName;
        private MotorType type;
        private DcMotor motor;
        private DcMotorEx motorEx;

        // Control parameters - PUBLIC for direct editing
        public double ticksPerRev = 112.0;
        public double maxRPM = 1400.0;
        public double kV = 0.0006785714285714286;
        public double kS = 0.06;
        public double kP = 0.0004;
        public double kI = 0.0002;
        public double kD = 0.00005;
        public double integralLimit = 0.2;

        // PID state - PUBLIC for direct access
        public double targetRPM = 0.0;
        public double integral = 0.0;
        public double lastError = 0.0;
        private long lastUpdateTime = 0;

        // Flag for velocity control mode - PUBLIC
        public boolean useVelocityControl = false;

        public MotorClass(String name, MotorType type) {
            this.hardwareMapName = name;
            this.type = type;
        }

        /**
         * Simple initialization - just hardware map.
         * Uses default settings and velocity control OFF.
         */
        public void init(HardwareMap hwMap) {
            init(hwMap, false, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        /**
         * Full initialization with all settings in one line.
         * @param hwMap Hardware map
         * @param useVelocityControl Enable velocity control (requires EXTENDED_MOTOR)
         * @param direction Motor direction
         * @param zeroPowerBehavior Zero power behavior
         * @param runMode Run mode
         */
        public void init(HardwareMap hwMap, boolean useVelocityControl,
                         DcMotorSimple.Direction direction,
                         DcMotor.ZeroPowerBehavior zeroPowerBehavior,
                         DcMotor.RunMode runMode) {
            if (type == MotorType.BASIC_MOTOR) {
                motor = hwMap.get(DcMotor.class, hardwareMapName);
            } else {
                motorEx = hwMap.get(DcMotorEx.class, hardwareMapName);
                motor = motorEx;
            }

            motor.setDirection(direction);
            motor.setZeroPowerBehavior(zeroPowerBehavior);
            motor.setMode(runMode);

            this.useVelocityControl = useVelocityControl;
            lastUpdateTime = System.nanoTime();
        }

        /**
         * Full initialization with motor parameters and control gains.
         * Use this for complete one-line setup.
         */
        public void init(HardwareMap hwMap, boolean useVelocityControl,
                         DcMotorSimple.Direction direction,
                         DcMotor.ZeroPowerBehavior zeroPowerBehavior,
                         DcMotor.RunMode runMode,
                         double ticksPerRev, double maxRPM,
                         double kV, double kS,
                         double kP, double kI, double kD,
                         double integralLimit) {
            // Initialize hardware first
            init(hwMap, useVelocityControl, direction, zeroPowerBehavior, runMode);

            // Set all parameters
            this.ticksPerRev = ticksPerRev;
            this.maxRPM = maxRPM;
            this.kV = kV;
            this.kS = kS;
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.integralLimit = integralLimit;
        }

        /**
         * Sets motor to a power level (-1.0 to 1.0).
         * If velocity control is enabled, this will be ignored in update().
         */
        public void setPower(double power) {
            if (motor != null) {
                motor.setPower(Math.max(-1.0, Math.min(1.0, power)));
            }
        }

        /**
         * Sets target RPM for velocity control.
         * Only works if motor is EXTENDED_MOTOR and useVelocityControl is true.
         * Can also directly set: motor.targetRPM = 980.0;
         */
        public void setTargetRPM(double rpm) {
            this.targetRPM = Math.max(0, Math.min(maxRPM, rpm));
        }

        /**
         * Enables or disables velocity control mode.
         * When enabled, call update() in your loop.
         * Can also directly set: motor.useVelocityControl = true;
         */
        public void enableVelocityControl(boolean enable) {
            this.useVelocityControl = enable;
            if (!enable) {
                integral = 0.0;
                lastError = 0.0;
            }
        }

        /**
         * Updates the velocity control loop.
         * Call this every loop iteration if velocity control is enabled.
         */
        public void update() {
            if (!useVelocityControl || motorEx == null) {
                return;
            }

            // Calculate dt
            long currentTime = System.nanoTime();
            double dt = (currentTime - lastUpdateTime) / 1_000_000_000.0;
            if (dt < 0.001) dt = 0.001;
            lastUpdateTime = currentTime;

            // Get current velocity
            double velocityTicksPerSec = motorEx.getVelocity();
            double currentRPM = (velocityTicksPerSec / ticksPerRev) * 60.0;

            // Feedforward
            double ff = 0.0;
            if (targetRPM > 20) {
                ff = kS + kV * targetRPM;
            }

            // PID
            double error = targetRPM - currentRPM;
            integral += error * dt;
            integral = Math.max(-integralLimit, Math.min(integralLimit, integral));
            double derivative = (error - lastError) / dt;
            lastError = error;
            double pid = kP * error + kI * integral + kD * derivative;

            // Output
            double output = ff + pid;
            output = Math.max(-1.0, Math.min(1.0, output));
            motorEx.setPower(output);
        }

        /**
         * Stops the motor.
         */
        public void stop() {
            if (motor != null) {
                motor.setPower(0);
            }
            targetRPM = 0.0;
            integral = 0.0;
            lastError = 0.0;
        }

        /**
         * Gets the current encoder position.
         */
        public int getCurrentPosition() {
            return motor != null ? motor.getCurrentPosition() : 0;
        }

        /**
         * Gets the current RPM (requires EXTENDED_MOTOR).
         */
        public double getCurrentRPM() {
            if (motorEx != null) {
                double velocityTicksPerSec = motorEx.getVelocity();
                return (velocityTicksPerSec / ticksPerRev) * 60.0;
            }
            return 0.0;
        }

        /**
         * Checks if motor is at target RPM within tolerance.
         */
        public boolean isAtTarget(double toleranceRPM) {
            return Math.abs(getCurrentRPM() - targetRPM) < toleranceRPM;
        }

        // Configuration methods
        public void setDirection(DcMotorSimple.Direction direction) {
            if (motor != null) motor.setDirection(direction);
        }

        public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
            if (motor != null) motor.setZeroPowerBehavior(behavior);
        }

        public void setMode(DcMotor.RunMode mode) {
            if (motor != null) {
                motor.setMode(mode);
                if (mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                    integral = 0.0;
                    lastError = 0.0;
                }
            }
        }

        public void setTicksPerRev(double ticks) { this.ticksPerRev = ticks; }
        public void setMaxRPM(double rpm) { this.maxRPM = rpm; }
        public void setPIDGains(double kp, double ki, double kd) {
            this.kP = kp;
            this.kI = ki;
            this.kD = kd;
        }
        public void setFeedforwardGains(double kv, double ks) {
            this.kV = kv;
            this.kS = ks;
        }
        public void setAllGains(double kv, double ks, double kp, double ki, double kd) {
            this.kV = kv;
            this.kS = ks;
            this.kP = kp;
            this.kI = ki;
            this.kD = kd;
        }
        public void setIntegralLimit(double limit) { this.integralLimit = limit; }

        // Getters (most values are public fields now)
        public String getName() { return hardwareMapName; }
        public MotorType getType() { return type; }
        public DcMotor getMotor() { return motor; }
        public DcMotorEx getMotorEx() { return motorEx; }
    }