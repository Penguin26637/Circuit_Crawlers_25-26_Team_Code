package org.firstinspires.ftc.teamcode.PreComp2Files;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="DriverPractice", group="Drive Train")
public class DriverPractice extends LinearOpMode {

    // --- Gamepad 1 drive motors ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // --- Wheel brake ---
    private boolean wheelBreak = false;
    private int wheelBreakTargetFL, wheelBreakTargetFR, wheelBreakTargetBL, wheelBreakTargetBR;
    private static final double wheelBreak_kP = 0.01;
    private static final double wheelBreak_maxPower = 0.2;
    private static final int wheelBreak_maxError = 100;

    // --- Runtime ---
    private final ElapsedTime runtime = new ElapsedTime();

    private boolean slow_mode = false;
    private boolean robot_centric = true;
    private boolean field_centric = false;


    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) result = Math.min(result, voltage);
        }
        return result;
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // --- Hardware Mapping ---
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backl");
        backRightDrive = hardwareMap.get(DcMotor.class, "backr");

        // --- Motor directions ---
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Waiting for start...");
        telemetry.addLine("Use Config HI");
        telemetry.update();
        waitForStart();
        runtime.reset();

        double nerf = 0.1;

        // Retrieve the IMU from the hardware map
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);

        while (opModeIsActive()) {
            double batteryVoltage = getBatteryVoltage();

            double Logdrive = -gamepad1.left_stick_y * nerf;
            double LATdrive = -gamepad1.left_stick_x * nerf;
            double Turndrive = -gamepad1.right_stick_x * nerf;

            // --- Wheel brake toggle ---
//            if (gamepad1.left_stick_button && gamepad1.right_stick_button && !wheelBreak) {
//                wheelBreak = true;
//                sleep(200);
//
//                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//                frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                frontLeftDrive.setTargetPosition(0);
//                frontRightDrive.setTargetPosition(0);
//                backLeftDrive.setTargetPosition(0);
//                backRightDrive.setTargetPosition(0);
//
//                frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                wheelBreakTargetFL = frontLeftDrive.getCurrentPosition();
//                wheelBreakTargetFR = frontRightDrive.getCurrentPosition();
//                wheelBreakTargetBL = backLeftDrive.getCurrentPosition();
//                wheelBreakTargetBR = backRightDrive.getCurrentPosition();

//            } else if (gamepad1.left_stick_button && gamepad1.right_stick_button && wheelBreak) {
//                wheelBreak = false;
//                sleep(200);
//            }

            // Slow mode toggle
            if(!slow_mode && gamepad1.right_bumper){
                nerf = 0.1;
                slow_mode = true;
                sleep(200);
//            } else if (slow_mode && gamepad1.right_bumper) {
//                nerf = 0.75;
//                slow_mode = false;
//                sleep(200);
//            }

            // --- Wheel brake control ---
//            if (wheelBreak) {
//                applyWheelBrake(frontLeftDrive, wheelBreakTargetFL);
//                applyWheelBrake(frontRightDrive, wheelBreakTargetFR);
//                applyWheelBrake(backLeftDrive, wheelBreakTargetBL);
//                applyWheelBrake(backRightDrive, wheelBreakTargetBR);

                telemetry.addData("WHEEL BRAKE ACTIVE", "True");
            } else if (!wheelBreak && robot_centric) {
                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                backLeftDrive.setPower(Logdrive - LATdrive + Turndrive);
                backRightDrive.setPower(-Logdrive + LATdrive + Turndrive);
                frontLeftDrive.setPower(Logdrive + LATdrive + Turndrive * nerf);
                frontRightDrive.setPower(-Logdrive - LATdrive + Turndrive * nerf);
            }
//            else if (!wheelBreak && field_centric) {
//
//                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//                frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//                double y = -gamepad1.left_stick_y * nerf;
//                double x = gamepad1.left_stick_x * nerf;
//                double rx = gamepad1.right_stick_x * nerf;
//
//                // Reset IMU yaw with start button
//                if (gamepad1.start) {
//                    imu.resetYaw();
//                }
//
////                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//                // Rotate the movement direction counter to the bot's rotation
////                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
////                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
////                rotX = rotX * 1.1;  // Counteract imperfect strafing
////
////                // Denominator is the largest motor power (absolute value) or 1
////                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
////                double frontLeftPower = (rotY + rotX + rx) / denominator;
////                double backLeftPower = (rotY - rotX + rx) / denominator;
////                double frontRightPower = (rotY - rotX - rx) / denominator;
////                double backRightPower = (rotY + rotX - rx) / denominator;
//
//                frontLeftDrive.setPower(frontLeftPower);
//                backLeftDrive.setPower(backLeftPower);
//                frontRightDrive.setPower(frontRightPower);
//                backRightDrive.setPower(backRightPower);
//            }

            // --- Telemetry ---
            telemetry.addData("Wheel Brake Active", wheelBreak);
            telemetry.addData("Front Left Encoder", frontLeftDrive.getCurrentPosition());
            telemetry.addData("Front Right Encoder", frontRightDrive.getCurrentPosition());
            telemetry.addData("Back Left Encoder", backLeftDrive.getCurrentPosition());
            telemetry.addData("Back Right Encoder", backRightDrive.getCurrentPosition());
            telemetry.addData("Slow Mode", slow_mode);
            telemetry.addData("Drive Mode", robot_centric ? "Robot-Centric" : "Field-Centric");
            telemetry.addData("Battery Voltage (V)", "%.2f", batteryVoltage);
            telemetry.addData("Runtime", "%.2f", runtime.seconds());

            telemetry.update();
            sleep(20);
        }
    }

    private void applyWheelBrake(DcMotor motor, int target) {
        int error = target - motor.getCurrentPosition();
        error = Math.max(-wheelBreak_maxError, Math.min(wheelBreak_maxError, error));
        double power = wheelBreak_kP * error;
        power = Math.max(-wheelBreak_maxPower, Math.min(wheelBreak_maxPower, power));
        motor.setPower(power);
    }
}