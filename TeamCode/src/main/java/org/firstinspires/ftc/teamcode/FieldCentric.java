package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled
@TeleOp(name = "FieldCentric", group = "Drive Train")
public class FieldCentric extends LinearOpMode {

    // Define motors                                
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private IMU imu = null;

    @Override
    public void runOpMode() {

        // Initialize hardware
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontl");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontr");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backl");
        backRightMotor = hardwareMap.get(DcMotor.class, "backr");

        // Reverse the right motors (or left, depending on robot build)
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Ensure all motors run with consistent directionality

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot (e.g., logo up, USB forward)
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Reset yaw angle if a specific button is pressed (e.g., 'options' or 'start' on Xbox-style controller)
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // Read bot heading (yaw) in radians
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double botHeading = orientation.getYaw(AngleUnit.RADIANS);

            // Get the gamepad inputs
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value, so we invert
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Rotate the movement direction counter to the bot's rotation
            double rotatedX = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
            double rotatedY = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(yaw), 1);
            double frontLeftPower = (rotatedY + rotatedX + yaw) / denominator;
            double backLeftPower = (rotatedY - rotatedX + yaw) / denominator;
            double frontRightPower = (rotatedY - rotatedX - yaw) / denominator;
            double backRightPower = (rotatedY + rotatedX - yaw) / denominator;

            // Set motor powers
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("Status", "Running");
            telemetry.addData("Bot Heading (Radians)", botHeading);
            telemetry.update();
        }
    }
}
