package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.OT;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp Demo for ServoClass
 *
 * Controls:
 * Gamepad 1:
 *   A - Close claw (position 1.0)
 *   B - Open claw (position 0.0)
 *   X - Half close claw (position 0.5)
 *   Y - Quarter close claw (position 0.25)
 *
 *   DPAD_UP - Wrist up (position 0.8)
 *   DPAD_DOWN - Wrist down (position 0.2)
 *   DPAD_LEFT - Wrist middle (position 0.5)
 *
 *   LEFT_BUMPER - Intake reverse (power -1.0)
 *   RIGHT_BUMPER - Intake forward (power 1.0)
 *   LEFT_TRIGGER - Intake slow reverse (power -0.5)
 *   RIGHT_TRIGGER - Intake slow forward (power 0.5)
 *   (Release all buttons to stop intake)
 */
@TeleOp(name = "Servo Demo - TeleOp", group = "Demo")
public class ServoTeleOpDemo extends OpMode {

    // Servo objects
    private org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.OT.ServoClass claw;
    private org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.OT.ServoClass wrist;
    private org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.OT.ServoClass intake;

    @Override
    public void init() {
        // Initialize servos
        claw = new org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.OT.ServoClass("claw", org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.OT.ServoClass.ServoType.STANDARD_SERVO);
        wrist = new org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.OT.ServoClass("wrist", org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.OT.ServoClass.ServoType.CONTINUOUS_SERVO);
        intake = new org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.OT.ServoClass("i", org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.OT.ServoClass.ServoType.CONTINUOUS_SERVO);

        // Initialize hardware
        claw.init(hardwareMap);
        wrist.init(hardwareMap);
        intake.init(hardwareMap);

        // Set initial positions
        claw.goToPosition(0.0);   // Start with claw open
        wrist.goToPosition(0.5);  // Start with wrist in middle
        intake.stop();            // Start with intake stopped

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Claw", "%.2f", claw.getCurrentPosition());
        telemetry.addData("Wrist", "%.2f", wrist.getCurrentPosition());
        telemetry.update();
    }

//    private int loopCount = 0;

    @Override
    public void loop() {
//        loopCount++;

        // Claw controls (A, B, X, Y buttons)
        if (gamepad1.a) {
            claw.goToPosition(1.0);  // Close
        } else if (gamepad1.b) {
            claw.goToPosition(0.0);  // Open
        } else if (gamepad1.x) {
            claw.goToPosition(0.5);  // Half close
        } else if (gamepad1.y) {
            claw.goToPosition(0.25); // Quarter close
        }

        // Wrist controls (DPAD)
        if (gamepad1.dpad_up) {
            wrist.goToPosition(0.8);  // Up
        } else if (gamepad1.dpad_down) {
            wrist.goToPosition(0.2);  // Down
        } else if (gamepad1.dpad_left) {
            wrist.goToPosition(0.5);  // Middle
        }

        // Intake controls (Bumpers and Triggers)
        // Priority order: bumpers > triggers > stop
        if (gamepad1.left_bumper) {
            intake.goToPosition(-1.0);  // Full reverse
        } else if (gamepad1.right_bumper) {
            intake.goToPosition(1.0);   // Full forward
        } else if (gamepad1.left_trigger > 0.1) {
            intake.goToPosition(-0.5);  // Slow reverse
        } else if (gamepad1.right_trigger > 0.1) {
            intake.goToPosition(0.5);   // Slow forward
        } else {
            intake.stop();              // Stop when no buttons pressed
        }

        // Telemetry
        telemetry.addData("Status", "Running");
//        telemetry.addData("Loop Count", loopCount);
//        telemetry.addData("", "");
        telemetry.addData("Claw Position", "%.2f", claw.getCurrentPosition());
        telemetry.addData("Wrist Position", "%.2f", wrist.getCurrentPosition());
        telemetry.addData("", "");
        telemetry.addData("Controls", "");
        telemetry.addData(" A/B/X/Y", "Claw positions");
        telemetry.addData(" DPAD", "Wrist positions");
        telemetry.addData(" Bumpers/Triggers", "Intake power");
        telemetry.addData(" (Release all)", "Stop intake");
        telemetry.addData("Intake", intake.getCurrentPosition());
        telemetry.update();
    }
}