package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Shooter Motor Dashboard", group = "Test")
public class ShooterTeleOpSimple extends LinearOpMode {

    // ==================== DASHBOARD CONFIG ====================
    // All these values can be edited live in FTC Dashboard

    // Shooter Control
    public static double SHOOTER_TARGET_RPM = 0;
    public static double SHOOTER_RPM_INCREMENT = 50;
    public static double SHOOTER_TOLERANCE = 50;

    // PID Tuning - Can be adjusted live
    public static double SHOOTER_KP = 0.0006785714285714286;
    public static double SHOOTER_KI = 0.06;
    public static double SHOOTER_KD = 0.0004;
    public static double SHOOTER_KF = 0.0002;
    public static double SHOOTER_KV = 0.00005;

    // Motor Specifications
    public static double SHOOTER_TICKS_PER_REV = 112.0;
    public static double SHOOTER_MAX_RPM = 1400.0;

    // Quick Presets (optional)
    public static double PRESET_LOW = 600;
    public static double PRESET_MED = 900;
    public static double PRESET_HIGH = 1200;

    // ==================== INSTANCE VARIABLES ====================
    private ShooterClass shooter;
    private FtcDashboard dashboard;
    private MultipleTelemetry multiTelemetry;

    private boolean shooterEnabled = false;
    private boolean lastAButton = false;
    private boolean applyPIDChanges = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // ==================== SETUP ====================

        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        multiTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize ShooterClass (only shooter motor, no hinges or magazines)
        shooter = new ShooterClass(
                hardwareMap,
                multiTelemetry,
                true,      // Initialize shooter
                false,     // No hinge1
                false,     // No hinge2
                "none",    // No magazine 1
                "none",    // No magazine 2
                "none",    // No magazine 3
                "none"     // No magazine 4
        );

        // Apply initial PID values
        if (shooter.shooterInitialized) {
            shooter.shooterMotor.setTicksPerRev(SHOOTER_TICKS_PER_REV);
            shooter.shooterMotor.setMaxRpmUnderLoad(SHOOTER_MAX_RPM);
            shooter.shooterMotor.setAllGains(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF, SHOOTER_KV);
        }

        // Status
        multiTelemetry.addData("Status", "Initialized");
        multiTelemetry.addData("Shooter", shooter.shooterInitialized ? "Ready" : "FAILED");
        multiTelemetry.addLine();
        multiTelemetry.addLine("Controls:");
        multiTelemetry.addLine("A - Toggle Shooter On/Off");
        multiTelemetry.addLine("D-Pad Up/Down - Adjust RPM");
        multiTelemetry.addLine("X - Set Low Preset");
        multiTelemetry.addLine("Y - Set Med Preset");
        multiTelemetry.addLine("B - Set High Preset");
        multiTelemetry.addLine("Right Bumper - Apply PID Changes");
        multiTelemetry.update();

        waitForStart();

        // ==================== MAIN LOOP ====================
        while (opModeIsActive()) {

            // ==================== CONTROLS ====================

            // Toggle shooter on/off with A button
            boolean currentAButton = gamepad1.a;
            if (currentAButton && !lastAButton) {
                shooterEnabled = !shooterEnabled;
            }
            lastAButton = currentAButton;

            // Adjust target RPM with D-Pad
            if (gamepad1.dpad_up) {
                SHOOTER_TARGET_RPM += SHOOTER_RPM_INCREMENT;
            }
            if (gamepad1.dpad_down) {
                SHOOTER_TARGET_RPM -= SHOOTER_RPM_INCREMENT;
                if (SHOOTER_TARGET_RPM < 0) SHOOTER_TARGET_RPM = 0;
            }

            // Quick presets
            if (gamepad1.x) {
                SHOOTER_TARGET_RPM = PRESET_LOW;
            }
            if (gamepad1.y) {
                SHOOTER_TARGET_RPM = PRESET_MED;
            }
            if (gamepad1.b) {
                SHOOTER_TARGET_RPM = PRESET_HIGH;
            }

            // Reset to zero with left bumper
            if (gamepad1.left_bumper) {
                SHOOTER_TARGET_RPM = 0;
            }

            // Apply PID changes from dashboard with right bumper
            if (gamepad1.right_bumper) {
                if (shooter.shooterInitialized) {
                    shooter.shooterMotor.setTicksPerRev(SHOOTER_TICKS_PER_REV);
                    shooter.shooterMotor.setMaxRpmUnderLoad(SHOOTER_MAX_RPM);
                    shooter.shooterMotor.setAllGains(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF, SHOOTER_KV);
                    applyPIDChanges = true;
                }
            } else {
                applyPIDChanges = false;
            }

            // ==================== UPDATE SHOOTER ====================

            // Set target RPM
            shooter.shooterTargetRPM = SHOOTER_TARGET_RPM;

            // Update shooter
            shooter.update(shooterEnabled, false, false, false);

            // ==================== TELEMETRY ====================

            multiTelemetry.addData("=== STATUS ===", "");
            multiTelemetry.addData("Shooter Enabled", shooterEnabled ? "ON" : "OFF");
            multiTelemetry.addData("At Target", shooter.getShooterAtTarget(SHOOTER_TOLERANCE) ? "YES" : "NO");
            multiTelemetry.addLine();

            multiTelemetry.addData("=== SHOOTER ===", "");
            multiTelemetry.addData("Target RPM", "%.0f", SHOOTER_TARGET_RPM);
            multiTelemetry.addData("Current RPM", "%.1f", shooter.getShooterCurrentRPM());
            multiTelemetry.addData("Error", "%.1f RPM", SHOOTER_TARGET_RPM - shooter.getShooterCurrentRPM());
            multiTelemetry.addData("Tolerance", "±%.0f RPM", SHOOTER_TOLERANCE);
            multiTelemetry.addLine();

            multiTelemetry.addData("=== PID VALUES ===", "");
            multiTelemetry.addData("Kp", "%.10f", SHOOTER_KP);
            multiTelemetry.addData("Ki", "%.10f", SHOOTER_KI);
            multiTelemetry.addData("Kd", "%.10f", SHOOTER_KD);
            multiTelemetry.addData("Kf", "%.10f", SHOOTER_KF);
            multiTelemetry.addData("Kv", "%.10f", SHOOTER_KV);
            if (applyPIDChanges) {
                multiTelemetry.addData("PID Status", "APPLIED!");
            }
            multiTelemetry.addLine();

            multiTelemetry.addData("=== MOTOR SPECS ===", "");
            multiTelemetry.addData("Ticks/Rev", "%.1f", SHOOTER_TICKS_PER_REV);
            multiTelemetry.addData("Max RPM", "%.1f", SHOOTER_MAX_RPM);
            multiTelemetry.addLine();

            multiTelemetry.addData("=== PRESETS ===", "");
            multiTelemetry.addData("Low (X)", "%.0f RPM", PRESET_LOW);
            multiTelemetry.addData("Med (Y)", "%.0f RPM", PRESET_MED);
            multiTelemetry.addData("High (B)", "%.0f RPM", PRESET_HIGH);
            multiTelemetry.addLine();

            multiTelemetry.addData("=== CONTROLS ===", "");
            multiTelemetry.addData("A", "Toggle On/Off");
            multiTelemetry.addData("D-Pad", "Adjust RPM (±%.0f)", SHOOTER_RPM_INCREMENT);
            multiTelemetry.addData("X/Y/B", "Presets");
            multiTelemetry.addData("Left Bumper", "Reset to 0");
            multiTelemetry.addData("Right Bumper", "Apply PID Changes");

            multiTelemetry.update();

            // Small delay to prevent overwhelming the system
            sleep(20);
        }

        // ==================== CLEANUP ====================
        if (shooter.shooterInitialized) {
            shooter.shooterMotor.stop();
        }
    }
}