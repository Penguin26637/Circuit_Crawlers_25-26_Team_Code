package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem.ClassTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem.ShooterClass;

@TeleOp(name = "ShooterClassTest", group = "Test")
public class ShooterClassTest extends LinearOpMode {

    private ShooterClass shooter;

    // Debounce flags
    private boolean previousLeftBumper = false;

    // Toggle states
    private boolean shooterHingeUp = false;

    @Override
    public void runOpMode() {

        // Initialize ShooterClass: shooter motor = true, hinge = true, magazine1 = "crservo", magazine2 = "none"
        shooter = new ShooterClass(hardwareMap, telemetry, true, true, true, "CRservo", "none", "none", "none");

        telemetry.addData("Status", "Init complete");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // -------------------- Shooter Motor & Magazine --------------------
            shooter.shooterEnabled = gamepad2.right_trigger > 0.1;
            if (shooter.shooterEnabled) {
                shooter.shooterTargetRPM = 1200;   // example target RPM
                shooter.magazineTargetPower = 1.0;
                shooter.magazineEnabled = true;
            } else {
                shooter.magazineEnabled = false;
            }

            // -------------------- Hinge Toggle --------------------
            boolean currentLeftBumper = gamepad2.left_bumper;
            if (currentLeftBumper && !previousLeftBumper) {
                shooterHingeUp = !shooterHingeUp;
            }
            previousLeftBumper = currentLeftBumper;

            shooter.hinge1Enabled = true;
            shooter.hinge1TargetPosition = shooterHingeUp ? 1.0 : 0.0;
            shooter.hinge2TargetPosition = shooterHingeUp ? 1.0 : 0.0;

            // these values may have to get changes to variables pulled from the main teleop

            // -------------------- Update Shooter --------------------
            shooter.update(shooter.shooterEnabled, shooter.hinge1Enabled, shooter.hinge2Enabled, shooter.magazineEnabled);

            // -------------------- Telemetry --------------------
            telemetry.addData("Shooter RPM", shooter.getShooterCurrentRPM());
            telemetry.addData("Shooter Target RPM", shooter.shooterTargetRPM);
            telemetry.addData("Shooter Hinge", shooterHingeUp ? "Up" : "Down");
            telemetry.addData("Magazine", shooter.magazineEnabled ? "On" : "Off");
            telemetry.update();
        }
    }
}
