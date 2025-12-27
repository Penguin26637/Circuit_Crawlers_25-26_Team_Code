package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem.ClassTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem.IntakeClass;

@TeleOp(name = "IntakeClassTest", group = "Test")
public class IntakeClassTest extends LinearOpMode {

    private IntakeClass intake;
    private boolean intakeIn = false;       // True = intake in, False = out/stop
    private boolean previousA = false;      // Tracks previous button state

    @Override
    public void runOpMode() {
        // Initialize IntakeClass: assuming intake1 is a CRServo, intake2 none
        intake = new IntakeClass(hardwareMap, telemetry, "crservo", "none");

        telemetry.addData("Status", "Init complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // -------------------- Button Debounce Logic --------------------
            boolean currentA = gamepad1.a;

            if (currentA && !previousA) {  // Button pressed (rising edge)
                intakeIn = !intakeIn;       // Toggle intake direction
            }

            previousA = currentA;  // Save current state for next loop

            // -------------------- Update Intake --------------------
            if (intakeIn) {
                intake.targetPower = 1.0;  // Intake in
            } else {
                intake.targetPower = -1.0; // Intake out (reverse)
            }

            intake.update(true);

            // -------------------- Telemetry --------------------
            telemetry.addData("Intake State", intakeIn ? "In" : "Out");
            telemetry.addData("Intake Power", intake.getIntake1Value());
            telemetry.update();
        }

        intake.stopAll();
    }
}
