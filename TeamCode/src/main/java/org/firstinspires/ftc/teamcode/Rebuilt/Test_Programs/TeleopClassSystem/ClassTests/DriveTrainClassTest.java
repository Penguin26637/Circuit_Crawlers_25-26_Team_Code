package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem.ClassTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem.DriveControlClass;

@TeleOp(name="DriveTrainClassTest", group="Drive Train")
public class DriveTrainClassTest extends LinearOpMode {

    private DriveControlClass drive;
    private final ElapsedTime runtime = new ElapsedTime();

    private boolean slowMode = false;
    private boolean wheelBrakeActive = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing DriveControlClass...");
        telemetry.update();

        drive = new DriveControlClass(hardwareMap, telemetry, true, true, true);

        telemetry.addData("Status", "Initialization complete");
        telemetry.addLine("Press start to drive");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double nerf = 0.75;

        while (opModeIsActive()) {

            // --- Toggle slow mode ---
            if (!slowMode && gamepad1.right_bumper) {
                nerf = 0.1;
                slowMode = true;
                sleep(200);
            } else if (slowMode && gamepad1.right_bumper) {
                nerf = 0.75;
                slowMode = false;
                sleep(200);
            }

            // --- Toggle wheel brake ---
            if (gamepad1.left_stick_button && gamepad1.right_stick_button && !wheelBrakeActive) {
                wheelBrakeActive = true;
                drive.initWheelBrake();
                sleep(200);
            } else if (gamepad1.left_stick_button && gamepad1.right_stick_button && wheelBrakeActive) {
                wheelBrakeActive = false;
                sleep(200);
            }

            // --- Set DriveControlClass flags ---
            drive.useWheelBrake = wheelBrakeActive;
            drive.useFieldCentric = false; // modify if you want field-centric
            drive.nerf = nerf;

            // --- Read inputs ---
            double forward = -gamepad1.left_stick_y * nerf;
            double strafe = -gamepad1.left_stick_x * nerf;
            double turn = -gamepad1.right_stick_x * nerf;

            // --- Update drive ---
            drive.update(true, forward, strafe, turn, 0);

            // --- Telemetry ---
            telemetry.addData("Wheel Brake", wheelBrakeActive);
            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("Front Left Encoder", drive.frontLeftDrive.getCurrentPosition());
            telemetry.addData("Front Right Encoder", drive.frontRightDrive.getCurrentPosition());
            telemetry.addData("Back Left Encoder", drive.backLeftDrive.getCurrentPosition());
            telemetry.addData("Back Right Encoder", drive.backRightDrive.getCurrentPosition());
            telemetry.addData("Runtime", "%.2f", runtime.seconds());
            telemetry.update();

            sleep(20);
        }

        // Stop motors at the end
        drive.stopAllMotors();
    }
}
