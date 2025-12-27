package org.firstinspires.ftc.teamcode.PreComp2Files;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="Shooter_Test_Dash", group = "Testing")
public class Shooter_Test_Dash extends OpMode {

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
    private ElapsedTime dtTimer = new ElapsedTime();
    private double lastPosition = 0.0;
    private double integral = 0.0;
    private double lastError = 0.0;

    // Target flywheel speed in RPM
    public static double targetRPM = 980.0;  // Start at max for testing

    // Dashboard
    private FtcDashboard dashboard;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lastPosition = flywheel.getCurrentPosition();
        dtTimer.reset();

        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("=== RS-555 FLYWHEEL TUNED ===");
        telemetry.addLine("Motor: 6000 RPM no-load spec");
        telemetry.addLine("Measured: 1400 RPM at full power");
        telemetry.addLine("Load factor: 23% (heavy flywheel)");
        telemetry.addLine("");
        telemetry.addLine("System is now calibrated!");
        telemetry.addLine("Use FTC Dashboard to fine-tune PID gains");
        telemetry.update();
    }
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) result = Math.min(result, voltage);
        }
        return result;
    }

    @Override
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
        double currentRPM = (velocityTicksPerSec / TICKS_PER_REV) * 60.0;

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
        packet.put("Error %", (error / targetRPM) * 100.0);

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
        packet.put("FF % of Total", (ff / output) * 100.0);
        packet.put("battery_Voltage", batteryVoltage);
        packet.put("Encoder Ticks", pos);

        dashboard.sendTelemetryPacket(packet);

        // ======== 6. Gamepad Control ========
        // Adjust target RPM
        if (gamepad1.dpad_up) targetRPM += 100;
        if (gamepad1.dpad_down) targetRPM -= 100;

        // Quick presets
        if (gamepad1.y) targetRPM = MAX_RPM_UNDER_LOAD;  // Max
        if (gamepad1.x) targetRPM = MAX_RPM_UNDER_LOAD * 0.75;  // 75%
        if (gamepad1.b) targetRPM = MAX_RPM_UNDER_LOAD * 0.5;   // 50%
        if (gamepad1.a) targetRPM = 0;  // Stop

        // Reset integral on button press (useful if tuning)
        if (gamepad1.left_bumper) {
            integral = 0.0;
            telemetry.addLine("Integral Reset!");
        }

        // Clamp target to realistic range (0-1400 RPM)
        targetRPM = Math.max(0, Math.min(MAX_RPM_UNDER_LOAD, targetRPM));

        // ======== 7. Driver Station Telemetry ========
        telemetry.addData("🎯 Target", "%.0f RPM", targetRPM);
        telemetry.addData("⚡ Actual", "%.0f RPM (%.1f%%)",
                         currentRPM, (currentRPM/targetRPM)*100);
        telemetry.addData("🔋 Power", "%.2f", output);
        telemetry.addData("📊 Error", "%.0f RPM", error);
        telemetry.addLine();
        telemetry.addData("Max Under Load", "%.0f RPM", MAX_RPM_UNDER_LOAD);

        if (Math.abs(output) >= 0.99) {
            telemetry.addLine("⚠️ SATURATED - Reduce target or increase MAX_RPM_UNDER_LOAD");
        }

        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  D-Pad Up/Down: ±100 RPM");
        telemetry.addLine("  Y: Max | X: 75% | B: 50% | A: Stop");
        telemetry.addLine("  Left Bumper: Reset Integral");

        telemetry.update();
    }


}