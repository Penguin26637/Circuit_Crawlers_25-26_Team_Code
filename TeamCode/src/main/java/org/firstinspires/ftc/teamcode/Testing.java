package org.firstinspires.ftc.teamcode;

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
public class Testing {

    // ============= Motor Constants ============
    // RS-555 with 28 PPR encoder, quadrature = 28 * 4 = 112 ticks/rev ✓
    public static final double TICKS_PER_REV = 112.0;

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
    private ElapsedTime dtTimer = new ElapsedTime();
    private double lastPosition = 0.0;
    private double integral = 0.0;
    private double lastError = 0.0;
    private double currentRPM = 0.0;

    // Target flywheel speed in RPM
    public static double targetRPM = 980.0;

    // Dashboard
    private FtcDashboard dashboard;

    // ============= Constructor ============
    /**
     * Creates a new motor power regulator for flywheel control
     * @param hardwareMap Reference to OpMode's hardwareMap
     * @param telemetry Reference to OpMode's telemetry
     */
    public Testing(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    // ============= Initialization ============
    /**
     * Initialize the flywheel motor. Call this in your OpMode's init() method.
     */
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        lastPosition = flywheel.getCurrentPosition();
        dtTimer.reset();

        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("=== RS-555 FLYWHEEL TUNED ===");
        telemetry.addLine("Motor: 6000 RPM no-load spec");
        telemetry.addLine("Measured: 1400 RPM at full power");
        telemetry.addLine("Load factor: 23% (heavy flywheel)");
        telemetry.addLine("");
        telemetry.addLine("System is now calibrated!");
        telemetry.addLine("Use FTC Dashboard to fine-tune PID gains");
        telemetry.update();
    }

    // ============= Public API ============

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
     * Get the current measured RPM
     * @return Current RPM from encoder
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
     * Get current battery voltage
     * @return Battery voltage in volts
     */
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) result = Math.min(result, voltage);
        }
        return result;
    }

    /**
     * Update the controller. Call this in your OpMode's loop() method.
     */
    public void update() {

        // ======== 1. Compute actual RPM from encoder ========
        double pos = flywheel.getCurrentPosition();
        double dt = dtTimer.seconds();

        // Prevent division by zero on first loop
        if (dt < 0.001) dt = 0.001;
        dtTimer.reset();

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
        packet.put("Target RPM", targetRPM);
        packet.put("Actual RPM", currentRPM);
        packet.put("Error RPM", error);
        packet.put("Error %", targetRPM != 0 ? (error / targetRPM) * 100.0 : 0.0);

        // Control outputs
        packet.put("Total Power", output);
        packet.put("FF Power", ff);
        packet.put("PID Power", pid);
        packet.put("Integral", integral);

        // Tuning parameters
        packet.put("--- TUNING PARAMS ---", "");
        packet.put("MAX_RPM_UNDER_LOAD", MAX_RPM_UNDER_LOAD);
        packet.put("kV", kV);
        packet.put("kS", kS);
        packet.put("kP", kP);
        packet.put("kI", kI);
        packet.put("kD", kD);

        double batteryVoltage = getBatteryVoltage();

        // Diagnostics
        packet.put("--- DIAGNOSTICS ---", "");
        packet.put("Saturated?", Math.abs(output) >= 0.99);
        packet.put("FF % of Total", output != 0 ? (ff / output) * 100.0 : 0.0);
        packet.put("Battery Voltage", batteryVoltage);
        packet.put("Encoder Ticks", pos);

        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * Add detailed telemetry to driver station
     */
    public void addTelemetry() {
        telemetry.addData("🎯 Target", "%.0f RPM", targetRPM);
        telemetry.addData("⚡ Actual", "%.0f RPM (%.1f%%)",
                currentRPM, targetRPM != 0 ? (currentRPM/targetRPM)*100 : 0);
        telemetry.addData("🔋 Power", "%.2f", flywheel.getPower());
        telemetry.addData("📊 Error", "%.0f RPM", targetRPM - currentRPM);
        telemetry.addLine();
        telemetry.addData("Max Under Load", "%.0f RPM", MAX_RPM_UNDER_LOAD);

        if (Math.abs(flywheel.getPower()) >= 0.99) {
            telemetry.addLine("⚠️ SATURATED - Reduce target or increase MAX_RPM_UNDER_LOAD");
        }
    }

    /**
     * Add simple telemetry to driver station
     */
    public void addSimpleTelemetry() {
        telemetry.addData("🎯 Target", "%.0f RPM", targetRPM);
        telemetry.addData("⚡ Actual", "%.0f RPM", currentRPM);
        telemetry.addData("At Target?", isAtTarget(50) ? "✓ YES" : "✗ NO");
    }
}