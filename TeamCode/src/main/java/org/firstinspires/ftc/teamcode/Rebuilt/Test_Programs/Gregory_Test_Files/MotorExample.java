package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Comprehensive example showing all ways to use MotorClass:
 * 1. One-line complete setup
 * 2. Simple setup + direct field editing
 * 3. Simple setup + method calls
 */
@TeleOp(name = "Motor Class All Options", group = "Test")
public class MotorExample extends LinearOpMode {

    // Configuration constants
    private static final double SHOOTER_TICKS_PER_REV = 112.0;
    private static final double SHOOTER_MAX_RPM = 1400.0;
    private static final double SHOOTER_KV = 0.0006785714285714286;
    private static final double SHOOTER_KS = 0.06;
    private static final double SHOOTER_KP = 0.0004;
    private static final double SHOOTER_KI = 0.0002;
    private static final double SHOOTER_KD = 0.00005;

    private static final double SHOOTER_ACTIVE_RPM = 980.0;
    private static final double SHOOTER_IDLE_RPM = 700.0;

    private MotorClass shooter1;
    private MotorClass shooter2;
    private MotorClass shooter3;
    private MotorClass driveMotor;

    @Override
    public void runOpMode() {
        telemetry.addLine("=== INITIALIZATION EXAMPLES ===");
        telemetry.update();
        sleep(1000);

        // ===================================================================
        // OPTION 1: Complete one-line setup (everything in init)
        // ===================================================================
        telemetry.addLine("Option 1: One-line complete setup");
        telemetry.update();

        shooter1 = new MotorClass("shooter", MotorClass.MotorType.EXTENDED_MOTOR);
        shooter1.init(hardwareMap, true,
                DcMotorSimple.Direction.REVERSE,
                DcMotor.ZeroPowerBehavior.FLOAT,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                SHOOTER_TICKS_PER_REV, SHOOTER_MAX_RPM,
                SHOOTER_KV, SHOOTER_KS,
                SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, 0.2);

        sleep(500);

        // ===================================================================
        // OPTION 2: Simple init + DIRECT FIELD EDITING (like your current system)
        // ===================================================================
        telemetry.addLine("Option 2: Simple init + direct editing");
        telemetry.update();

        shooter2 = new MotorClass("shooter", MotorClass.MotorType.EXTENDED_MOTOR);
        shooter2.init(hardwareMap, true,
                DcMotorSimple.Direction.REVERSE,
                DcMotor.ZeroPowerBehavior.FLOAT,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Direct field access - just like your DriveControlClass!
        shooter2.ticksPerRev = SHOOTER_TICKS_PER_REV;
        shooter2.maxRPM = SHOOTER_MAX_RPM;
        shooter2.kV = SHOOTER_KV;
        shooter2.kS = SHOOTER_KS;
        shooter2.kP = SHOOTER_KP;
        shooter2.kI = SHOOTER_KI;
        shooter2.kD = SHOOTER_KD;
        shooter2.integralLimit = 0.2;

        sleep(500);

        // ===================================================================
        // OPTION 3: Simple init + METHOD CALLS
        // ===================================================================
        telemetry.addLine("Option 3: Simple init + methods");
        telemetry.update();

        shooter3 = new MotorClass("shooter", MotorClass.MotorType.EXTENDED_MOTOR);
        shooter3.init(hardwareMap, true,
                DcMotorSimple.Direction.REVERSE,
                DcMotor.ZeroPowerBehavior.FLOAT,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter3.setTicksPerRev(SHOOTER_TICKS_PER_REV);
        shooter3.setMaxRPM(SHOOTER_MAX_RPM);
        shooter3.setAllGains(SHOOTER_KV, SHOOTER_KS, SHOOTER_KP, SHOOTER_KI, SHOOTER_KD);
        shooter3.setIntegralLimit(0.2);

        sleep(500);

        // ===================================================================
        // OPTION 4: Minimal setup for simple motor
        // ===================================================================
        telemetry.addLine("Option 4: Minimal for basic motor");
        telemetry.update();

        driveMotor = new MotorClass("leftFront", MotorClass.MotorType.BASIC_MOTOR);
        driveMotor.init(hardwareMap);
        // That's it! Uses all defaults

        telemetry.clear();
        telemetry.addLine("All motors initialized!");
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("A: All shooters ACTIVE");
        telemetry.addLine("B: All shooters IDLE");
        telemetry.addLine("X: All shooters STOP");
        telemetry.addLine("Y: Change shooter2 RPM directly");
        telemetry.addLine("Left Stick Y: Drive motor");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ===================================================================
            // CONTROL EXAMPLES
            // ===================================================================

            // Standard control using methods
            if (gamepad1.a) {
                shooter1.setTargetRPM(SHOOTER_ACTIVE_RPM);
                shooter2.setTargetRPM(SHOOTER_ACTIVE_RPM);
                shooter3.setTargetRPM(SHOOTER_ACTIVE_RPM);
            }
            if (gamepad1.b) {
                shooter1.setTargetRPM(SHOOTER_IDLE_RPM);
                shooter2.setTargetRPM(SHOOTER_IDLE_RPM);
                shooter3.setTargetRPM(SHOOTER_IDLE_RPM);
            }
            if (gamepad1.x) {
                shooter1.stop();
                shooter2.stop();
                shooter3.stop();
            }

            // DIRECT field editing example - set RPM directly!
            if (gamepad1.y) {
                shooter2.targetRPM = 1200.0;  // Direct access like your current system
            }

            // Live tuning example - adjust gains on the fly
            if (gamepad1.dpad_up) {
                shooter2.kP += 0.0001;  // Direct field access for tuning
            }
            if (gamepad1.dpad_down) {
                shooter2.kP -= 0.0001;
            }

            // Update all shooters
            shooter1.update();
            shooter2.update();
            shooter3.update();

            // Simple power control for drive motor
            double drivePower = -gamepad1.left_stick_y;
            driveMotor.setPower(drivePower);

            // ===================================================================
            // TELEMETRY - showing both direct field access and method access
            // ===================================================================
            telemetry.addLine("=== SHOOTER 1 (One-line init) ===");
            telemetry.addData("Target", "%.0f RPM", shooter1.targetRPM);  // Direct field
            telemetry.addData("Current", "%.0f RPM", shooter1.getCurrentRPM());
            telemetry.addData("At Target?", shooter1.isAtTarget(50.0));

            telemetry.addLine();
            telemetry.addLine("=== SHOOTER 2 (Direct editing) ===");
            telemetry.addData("Target", "%.0f RPM", shooter2.targetRPM);
            telemetry.addData("Current", "%.0f RPM", shooter2.getCurrentRPM());
            telemetry.addData("kP (tunable)", "%.6f", shooter2.kP);  // Show direct field

            telemetry.addLine();
            telemetry.addLine("=== SHOOTER 3 (Method calls) ===");
            telemetry.addData("Target", "%.0f RPM", shooter3.targetRPM);
            telemetry.addData("Current", "%.0f RPM", shooter3.getCurrentRPM());

            telemetry.addLine();
            telemetry.addData("Drive Power", "%.2f", drivePower);
            telemetry.addData("Drive Position", driveMotor.getCurrentPosition());

            telemetry.update();
            sleep(20);
        }
    }
}