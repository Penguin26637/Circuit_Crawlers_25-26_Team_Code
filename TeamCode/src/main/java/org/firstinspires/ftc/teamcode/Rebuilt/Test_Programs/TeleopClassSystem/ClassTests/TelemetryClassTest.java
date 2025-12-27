package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem.ClassTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem.IntakeClass;
import org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem.OdometryClass;
import org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem.ShooterClass;
import org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem.TelemetryClass;

@TeleOp(name = "TelemetryClassTest", group = "Test")
public class TelemetryClassTest extends LinearOpMode {

    private TelemetryClass telemetryManager;

    // Subsystems
    private IntakeClass intake;
    private ShooterClass shooter;
    private OdometryClass odometry;

    // Debounce flags
    private boolean previousIntakeBumper = false;
    private boolean previousHingeA = false;

    // Toggle states
    private boolean intakeIn = false;
    private boolean shooterHingeUp = false;

    @Override
    public void runOpMode() {

        // -------------------- Initialize subsystems --------------------
        intake = new IntakeClass(hardwareMap, telemetry, "crservo", "none");
        shooter = new ShooterClass(hardwareMap, telemetry, true, true, true, "crservo", "none", "none", "none");
        odometry = new OdometryClass(hardwareMap, telemetry);

        // -------------------- Initialize TelemetryClass --------------------
        telemetryManager = new TelemetryClass(telemetry, hardwareMap);
        telemetryManager.intake = intake;
        telemetryManager.shooter = shooter;
        telemetryManager.odometry = odometry;
        // telemetryManager.drive = yourDriveClassInstance; // optional if available

        telemetry.addData("Status", "Init complete");
        telemetry.update();
        waitForStart();

        boolean dashboardEnabled = true;

        while (opModeIsActive()) {

            // -------------------- Intake Control --------------------
            boolean currentBumper = gamepad2.left_bumper;
            if (currentBumper && !previousIntakeBumper) {
                intakeIn = !intakeIn;
            }
            previousIntakeBumper = currentBumper;

            intake.targetPower = intakeIn ? 1.0 : -1.0;
            intake.update(true);

            // -------------------- Shooter Control --------------------
            shooter.shooterEnabled = gamepad2.right_trigger > 0.1;
            shooter.magazineEnabled = shooter.shooterEnabled;
            shooter.magazineTargetPower = shooter.shooterEnabled ? 1.0 : 0;

            // Hinge toggle with debounce
            boolean currentA = gamepad2.a;
            if (currentA && !previousHingeA) {
                shooterHingeUp = !shooterHingeUp;
            }
            previousHingeA = currentA;

            shooter.hinge1Enabled = true;
            shooter.hinge2Enabled = true;
            shooter.hinge1TargetPosition = shooterHingeUp ? 1.0 : 0.0;

            // Update shooter
            shooter.update(shooter.shooterEnabled, shooter.hinge1Enabled, shooter.hinge2Enabled, shooter.magazineEnabled);

            // -------------------- Odometry Update --------------------
            odometry.update(true);

            // -------------------- Update Telemetry --------------------
            telemetryManager.update(true, dashboardEnabled);
        }

        intake.stopAll();
    }
}
