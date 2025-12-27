package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem.ClassTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem.OdometryClass;

@TeleOp(name = "OdometryClassTest", group = "Test")
public class OdometryClassTest extends LinearOpMode {

    private OdometryClass odometry;

    @Override
    public void runOpMode() {

        // Initialize OdometryClass
        odometry = new OdometryClass(hardwareMap, telemetry);

        telemetry.addData("Status", odometry.getInitialized() ? "Odometry Initialized" : "Error initializing");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // -------------------- Update Odometry --------------------
            odometry.update(true);

            // -------------------- Reset Position --------------------
            if (gamepad1.back) {
                odometry.resetPosition();
            }

            // -------------------- Telemetry --------------------
            telemetry.addData("X Position (in)", odometry.xPos);
            telemetry.addData("Y Position (in)", odometry.yPos);
            telemetry.addData("Heading (deg)", odometry.getHeadingDegrees());
            telemetry.addData("Left Encoder", odometry.getLeftEncoder());
            telemetry.addData("Right Encoder", odometry.getRightEncoder());
            telemetry.addData("Perp Encoder", odometry.getPerpEncoder());
            telemetry.update();
        }
    }
}
