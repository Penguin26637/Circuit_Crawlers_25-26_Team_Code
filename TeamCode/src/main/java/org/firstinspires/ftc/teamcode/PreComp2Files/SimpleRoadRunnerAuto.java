package org.firstinspires.ftc.teamcode.PreComp2Files;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.PreComp2Files.ThreeDeadWheelLocalizer;
@Disabled
@Autonomous(name = "Simple Road Runner Auto", group = "Auto")
public class SimpleRoadRunnerAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Set starting pose (x, y, heading in radians)
        Pose2d beginPose = new Pose2d(-54, -46, 0);

        // Initialize Road Runner drive with 3-wheel odometry
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // CRITICAL: If you have ThreeDeadWheelLocalizer, uncomment this:
//         drive.localizer = new ThreeDeadWheelLocalizer(hardwareMap, drive.lazyImu.get());

        // IMPORTANT: Add telemetry to debug localization
        while (opModeInInit()) {
            telemetry.addData("Starting Pose", beginPose);
            telemetry.addData("Starting X", beginPose.position.x);
            telemetry.addData("Starting Y", beginPose.position.y);
            telemetry.addData("Starting Heading (deg)", Math.toDegrees(beginPose.heading.toDouble()));

            // Check if odometry pods are working
            telemetry.addData("", "--- Check These Match TeleOp ---");
            telemetry.addData("Left Odo (i)", hardwareMap.dcMotor.get("i").getCurrentPosition());
            telemetry.addData("Right Odo (i2)", hardwareMap.dcMotor.get("i2").getCurrentPosition());
            telemetry.addData("Back Odo (s)", hardwareMap.dcMotor.get("s").getCurrentPosition());
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        // TEST 1: Simple forward movement
        telemetry.addData("Status", "Running Test 1: Forward");
        telemetry.update();

        Pose2d afterTest1 = new Pose2d(-40, -46, 0);  // Expected pose after test

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .lineToX(-40)  // Drive forward 14 inches
                        .build()
        );

        telemetry.addData("Test 1 Complete", "Check if robot moved correctly");
        telemetry.addData("Expected X", -40);
        telemetry.addData("Expected Y", -46);
        telemetry.addData("", "Did it move 14 inches forward?");
        telemetry.addData("", "Press A to continue to full path");
        telemetry.update();

        // Wait for user confirmation
        while (opModeIsActive() && !gamepad1.a) {
            sleep(50);
        }

        // ===== FULL MEEP MEEP PATH =====
        telemetry.addData("Status", "Running Full Path");
        telemetry.update();

        Actions.runBlocking(
                drive.actionBuilder(afterTest1)  // Start from where test 1 ended
                        .splineTo(new Vector2d(-12, -25), Math.PI / 2)
                        .turn(Math.toRadians(180))
                        .lineToY(-50)
                        .splineTo(new Vector2d(-25,-28), Math.PI/2)
                        .turn(Math.toRadians(30))
                        .waitSeconds(3)
                        .splineTo(new Vector2d(10,-27), Math.PI/2)
                        .turn(Math.toRadians(180))
                        .lineToY(-50)
                        .splineTo(new Vector2d(-25,-28), Math.PI/2)
                        .turn(Math.toRadians(30))
                        .waitSeconds(3)
                        .splineTo(new Vector2d(-12,-35), Math.PI/2)
                        .build()
        );

        // ===== END PATH =====

        telemetry.addData("Status", "Auto Complete");
        telemetry.addData("", "Path finished - check final position");
        telemetry.update();

        sleep(5000); // Keep telemetry visible
    }
}