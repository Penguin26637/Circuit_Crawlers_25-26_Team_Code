package org.firstinspires.ftc.teamcode.PreComp2Files;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class random {

    // ============= Motor Constants ============
    // RS-555 with 28 PPR encoder, quadrature = 28 * 4 = 112 ticks/rev ✓
    public static double TICKS_PER_REV = 112.0;

    // Motor spec: 6000 RPM no-load @ 12V
    // MEASURED: 1400 RPM at full power with flywheel attached
    // This is your actual max - heavy flywheel causes significant load
    public static double MAX_RPM_UNDER_LOAD = 1400.0;  // Empirically determined

    // ============= Controller Gains (Tune These) ============
    // kV: At 1400 RPM, we want ~95% power (leaving 5% for PID correction)
    // Calculation: kV = 0.95 / 1400 = 0.000678
    public static double kV = 0.0006785714285714286;
    public static double kS = 0.06;  // Static friction baseline

    // PID gains - these should work well now that feedforward is correct
    public static double kP = 0.0004;  // Proportional gain
    public static double kI = 0.0002;  // Integral gain
    public static double kD = 0.00005; // Derivative gain

    // Anti-windup limit for integral term
    public static double integralLimit = 0.2;

    // ============= Internal State ============
    private DcMotorEx flywheel;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private String motorName;  // Store the motor name for reference
    private ElapsedTime dtTimer = new ElapsedTime();
    private double lastPosition = 0.0;
    private double integral = 0.0;
    private double lastError = 0.0;
    private double currentRPM = 0.0;

    // Target flywheel speed in RPM
    public static double targetRPM = 980.0;

    // Dashboard
    private FtcDashboard dashboard;

    /**
     * Create a motor power regulator with custom motor name
     * @param hardwareMap Reference to OpMode's hardwareMap
     * @param telemetry Reference to OpMode's telemetry
     * @param motorName Name of the motor in hardware configuration
     */
    public random(HardwareMap hardwareMap, Telemetry telemetry, String motorName) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.motorName = motorName;

        flywheel = hardwareMap.get(DcMotorEx.class, motorName);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        lastPosition = flywheel.getCurrentPosition();
        dtTimer.reset();

        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("=== " + motorName.toUpperCase() + " FLYWHEEL INITIALIZED ===");
        telemetry.addLine("Motor: 6000 RPM no-load spec");
        telemetry.addLine("Measured: 1400 RPM at full power");
        telemetry.addLine("System is now calibrated!");
        telemetry.update();
    }

    /**
     * Create a motor power regulator with default motor name "shooter"
     * @param hardwareMap Reference to OpMode's hardwareMap
     * @param telemetry Reference to OpMode's telemetry
     */
    public random(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, "shooter");
    }

    // ============= MOTOR CONTROL METHODS ============

    /**
     * Set the zero power behavior of the motor
     * @param zeroPowerBehavior Either FLOAT or BRAKE
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        flywheel.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * Get the current zero power behavior
     * @return Current zero power behavior
     */
    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return flywheel.getZeroPowerBehavior();
    }

    /**
     * Set the run mode of the motor
     * @param runMode RUN_USING_ENCODER, RUN_WITHOUT_ENCODER, RUN_TO_POSITION, or STOP_AND_RESET_ENCODER
     */
    public void setMode(DcMotor.RunMode runMode) {
        flywheel.setMode(runMode);

        // If resetting encoder, also reset our tracking variables
        if (runMode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            lastPosition = 0;
            integral = 0;
            lastError = 0;
            dtTimer.reset();
        }
    }

    /**
     * Get the current run mode
     * @return Current run mode
     */
    public DcMotor.RunMode getMode() {
        return flywheel.getMode();
    }

    /**
     * Set the motor direction
     * @param direction FORWARD or REVERSE
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        flywheel.setDirection(direction);
    }

    /**
     * Get the current motor direction
     * @return Current direction
     */
    public DcMotorSimple.Direction getDirection() {
        return flywheel.getDirection();
    }

    /**
     * Get direct access to the motor (use carefully - can break PID control)
     * @return The underlying DcMotorEx object
     */
    public DcMotorEx getMotor() {
        return flywheel;
    }

    // ============= SETTER METHODS FOR ALL PARAMETERS ============

    /**
     * Set the encoder ticks per revolution
     * @param ticksPerRev Encoder ticks per motor revolution
     */
    public void setTicksPerRev(double ticksPerRev) {
        TICKS_PER_REV = ticksPerRev;
    }

    /**
     * Get the current ticks per revolution setting
     * @return Ticks per revolution
     */
    public double getTicksPerRev() {
        return TICKS_PER_REV;
    }

    /**
     * Set the maximum RPM under load
     * @param maxRpm Maximum achievable RPM with flywheel attached
     */
    public void setMaxRpmUnderLoad(double maxRpm) {
        MAX_RPM_UNDER_LOAD = maxRpm;
    }

    /**
     * Get the maximum RPM under load
     * @return Maximum RPM under load
     */
    public double getMaxRpmUnderLoad() {
        return MAX_RPM_UNDER_LOAD;
    }

    /**
     * Set the velocity feedforward gain (kV)
     * @param kv Feedforward gain for velocity (power per RPM)
     */
    public void setKv(double kv) {
        kV = kv;
    }

    /**
     * Get the velocity feedforward gain (kV)
     * @return kV value
     */
    public double getKv() {
        return kV;
    }

    /**
     * Set the static friction compensation (kS)
     * @param ks Static friction baseline power
     */
    public void setKs(double ks) {
        kS = ks;
    }

    /**
     * Get the static friction compensation (kS)
     * @return kS value
     */
    public double getKs() {
        return kS;
    }

    /**
     * Set the proportional gain (kP)
     * @param kp Proportional gain
     */
    public void setKp(double kp) {
        kP = kp;
    }

    /**
     * Get the proportional gain (kP)
     * @return kP value
     */
    public double getKp() {
        return kP;
    }

    /**
     * Set the integral gain (kI)
     * @param ki Integral gain
     */
    public void setKi(double ki) {
        kI = ki;
    }

    /**
     * Get the integral gain (kI)
     * @return kI value
     */
    public double getKi() {
        return kI;
    }

    /**
     * Set the derivative gain (kD)
     * @param kd Derivative gain
     */
    public void setKd(double kd) {
        kD = kd;
    }

    /**
     * Get the derivative gain (kD)
     * @return kD value
     */
    public double getKd() {
        return kD;
    }

    /**
     * Set the target RPM for the flywheel
     * @param rpm Target RPM (will be clamped to 0-MAX_RPM_UNDER_LOAD)
     */
    public void setTargetRPM(double rpm) {
        targetRPM = Math.max(0, Math.min(MAX_RPM_UNDER_LOAD, rpm));
    }

    /**
     * Get the current target RPM
     * @return Target RPM
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Get the current measured RPM from the encoder
     * @return Current RPM
     */
    public double getCurrentRPM() {
        return currentRPM;
    }

    /**
     * Check if flywheel is within tolerance of target
     * @param toleranceRPM Tolerance in RPM
     * @return true if within tolerance
     */
    public boolean isAtTarget(double toleranceRPM) {
        return Math.abs(currentRPM - targetRPM) < toleranceRPM;
    }

    /**
     * Reset the integral accumulator. Useful if motor saturates.
     */
    public void resetIntegral() {
        integral = 0.0;
    }

    /**
     * Stop the flywheel and reset controller state
     */
    public void stop() {
        flywheel.setPower(0);
        targetRPM = 0;
        integral = 0;
    }

    /**
     * Set all PID gains at once
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    public void setPIDGains(double kp, double ki, double kd) {
        kP = kp;
        kI = ki;
        kD = kd;
    }

    /**
     * Set all feedforward gains at once
     * @param kv Velocity feedforward gain
     * @param ks Static friction compensation
     */
    public void setFeedforwardGains(double kv, double ks) {
        kV = kv;
        kS = ks;
    }

    /**
     * Set all controller parameters at once
     * @param kv Velocity feedforward gain
     * @param ks Static friction compensation
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    public void setAllGains(double kv, double ks, double kp, double ki, double kd) {
        kV = kv;
        kS = ks;
        kP = kp;
        kI = ki;
        kD = kd;
    }

    /**
     * Get the motor name
     * @return Hardware config name of the motor
     */
    public String getMotorName() {
        return motorName;
    }

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) result = Math.min(result, voltage);
        }
        return result;
    }


    public void loop() {

        // ======== 1. Compute actual RPM from encoder ========
        double pos = flywheel.getCurrentPosition();
        double dt = dtTimer.seconds();

        // Prevent division by zero on first loop
        if (dt < 0.001) dt = 0.001;
        dtTimer.reset();

        double deltaTicks = pos - lastPosition;
        lastPosition = pos;

        // Use built-in velocity measurement (more stable than manual calculation)
        double velocityTicksPerSec = flywheel.getVelocity();
        currentRPM = (velocityTicksPerSec / TICKS_PER_REV) * 60.0;

        // ======== 2. Compute Feedforward ========
        double ff = 0.0;
        if (targetRPM > 20) {
            // kS provides base power to overcome friction
            // kV scales linearly with RPM
            ff = kS + kV * targetRPM;
        }

        // ======== 3. Compute PID ========
        double error = targetRPM - currentRPM;

        // Integral with anti-windup clamping
        integral += error * dt;
        integral = Math.max(-integralLimit, Math.min(integralLimit, integral));

        double derivative = (error - lastError) / dt;
        lastError = error;

        double pid = kP * error + kI * integral + kD * derivative;

        // ======== 4. Combine Feedforward + PID ========
        double output = ff + pid;

        // Clamp to motor limits
        output = Math.max(-1.0, Math.min(1.0, output));

        // Command the motor
        flywheel.setPower(output);

        // ======== 5. Dashboard Telemetry ========
        TelemetryPacket packet = new TelemetryPacket();

        // Main metrics
        packet.put(motorName + " Target RPM", targetRPM);
        packet.put(motorName + " Actual RPM", currentRPM);
        packet.put(motorName + " Error RPM", error);
        packet.put(motorName + " Error %", targetRPM != 0 ? (error / targetRPM) * 100.0 : 0.0);

        // Control outputs
        packet.put(motorName + " Total Power", output);
        packet.put(motorName + " FF Power", ff);
        packet.put(motorName + " PID Power", pid);
        packet.put(motorName + " Integral", integral);

        // Tuning parameters
        packet.put("--- TUNING PARAMS ---", "");
        packet.put("TICKS_PER_REV", TICKS_PER_REV);
        packet.put("MAX_RPM_UNDER_LOAD", MAX_RPM_UNDER_LOAD);
        packet.put("kV", kV);
        packet.put("kS", kS);
        packet.put("kP", kP);
        packet.put("kI", kI);
        packet.put("kD", kD);
        double batteryVoltage = getBatteryVoltage();
        // Diagnostics
        packet.put("--- DIAGNOSTICS ---", "");
        packet.put(motorName + " Saturated?", Math.abs(output) >= 0.99);
        packet.put(motorName + " FF % of Total", output != 0 ? (ff / output) * 100.0 : 0.0);
        packet.put("battery_Voltage", batteryVoltage);
        packet.put(motorName + " Encoder Ticks", pos);
        packet.put(motorName + " Run Mode", flywheel.getMode().toString());
        packet.put(motorName + " Zero Power Behavior", flywheel.getZeroPowerBehavior().toString());

        dashboard.sendTelemetryPacket(packet);

        // Clamp target to realistic range (0-MAX_RPM_UNDER_LOAD)
        targetRPM = Math.max(0, Math.min(MAX_RPM_UNDER_LOAD, targetRPM));

        // ======== 6. Driver Station Telemetry ========
        telemetry.addData("🎯 " + motorName + " Target", "%.0f RPM", targetRPM);
        telemetry.addData("⚡ " + motorName + " Actual", "%.0f RPM (%.1f%%)",
                currentRPM, targetRPM != 0 ? (currentRPM/targetRPM)*100 : 0);
        telemetry.addData("🔋 " + motorName + " Power", "%.2f", output);
        telemetry.addData("📊 " + motorName + " Error", "%.0f RPM", error);
        telemetry.addLine();
        telemetry.addData("Max Under Load", "%.0f RPM", MAX_RPM_UNDER_LOAD);

        if (Math.abs(output) >= 0.99) {
            telemetry.addLine("⚠️ " + motorName + " SATURATED");
        }

        telemetry.update();
    }
}