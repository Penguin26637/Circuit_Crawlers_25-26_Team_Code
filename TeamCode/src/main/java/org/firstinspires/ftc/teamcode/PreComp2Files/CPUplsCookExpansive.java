package org.firstinspires.ftc.teamcode.PreComp2Files;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="CPUplsCookExpansive", group="Drive Train")
@Config
@Disabled
public class CPUplsCookExpansive extends LinearOpMode {

    // --- Gamepad 1 drive motors ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // --- Wheel brake ---
    public static boolean wheelBreak = false;
    public static int wheelBreakTargetFL, wheelBreakTargetFR, wheelBreakTargetBL, wheelBreakTargetBR;
    public static double wheelBreak_kP = 0.01;
    public static double wheelBreak_maxPower = 0.2;
    public static int wheelBreak_maxError = 100;

    // --- Odometry encoders (no motors attached, just encoder readings) ---
    private DcMotor intake, intake2, shooter;
    private DcMotorEx flywheel;
//    private Servo shooterHinge;
    private CRServo intakeToShooter, intakeToShooter2;

    public static boolean intakeActive = false;
    public static boolean shooterActive = false;
    public static boolean shooterUp = false;

    public static double intake_speed = 0.5;
    public static double shooter_power = 1;
    public static double intakeToShooter_power = 0.5;

    // --- Odometry constants ---
    public static double TICKS_PER_INCH = 337.2; // REV Odometry Pod 48mm wheel
    public static double TRACK_WIDTH = 13.5;    // distance between left/right wheels (inches)
    public static double BACK_WHEEL_OFFSET = 8; // distance from center (inches)

    private double xPos = 0, yPos = 0, heading = 0;
    private int prevLeft = 0, prevRight = 0, prevBack = 0;

    // --- Runtime ---
    private final ElapsedTime runtime = new ElapsedTime();

    // --- Dashboard ---
    private FtcDashboard dashboard;

    public static boolean slow_mode = false;
    public static boolean robot_centric = true;
    public static boolean field_centric = false;
    public static double kP = 0.01;      // Proportional control constant
    public static double maxPower = 0.2; // Maximum motor power (range: 0–1)
    public static int maxError = 100;

    public static final double TICKS_PER_REV = 112.0;

    // Motor spec: 6000 RPM no-load @ 12V
    // MEASURED: 1400 RPM at full power with shooter attached
    // This is your actual max - heavy shooter causes significant load
    public static double MAX_RPM_UNDER_LOAD = 1400.0;  // Empirically determined

    // ============= Controller Gains (Tune These) ============
    // kV: At 1400 RPM, we want ~95% power (leaving 5% for PID correction)
    // Calculation: kV = 0.95 / 1400 = 0.000678
    public static double kV = 0.0006785714285714286;
    public static double kS = 0.06;  // Static friction baseline

    // PID gains - these should work well now that feedforward is correct
    public static double kP_F = 0.0004;  // Proportional gain
    public static double kI = 0.0002;  // Integral gain
    public static double kD = 0.00005; // Derivative gain

    // Anti-windup limit for integral term
    public static double integralLimit = 0.2;

    // ============= Internal State ============

    private ElapsedTime dtTimer = new ElapsedTime();
    private double lastPosition = 0.0;
    private double integral = 0.0;
    private double lastError = 0.0;

    // Target shooter speed in RPM
    public static double targetRPM = 980.0;
    public static double currentRPM;
    public static double error;
    public static double output;
    public static double ff;
    public static double pid;

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) result = Math.min(result, voltage);
        }
        return result;
    }

    private void updateOdometry() {
        // --- Read encoder values ---
        int leftPos = -1*(intake.getCurrentPosition());
        int rightPos = (intake2.getCurrentPosition());
        int backPos = (shooter.getCurrentPosition());

        int deltaLeft = leftPos - prevLeft;
        int deltaRight = rightPos - prevRight;
        int deltaBack = backPos - prevBack;

        prevLeft = leftPos;
        prevRight = rightPos;
        prevBack = backPos;

        // --- Convert ticks to inches ---
        double dLeft = deltaLeft / TICKS_PER_INCH;
        double dRight = deltaRight / TICKS_PER_INCH;
        double dBack = deltaBack / TICKS_PER_INCH;

        // --- Odometry math ---
        double dHeading = (dRight - dLeft) / TRACK_WIDTH;
        double dForward = (dLeft + dRight) / 2.0;
        double dSide = dBack - (dHeading * BACK_WHEEL_OFFSET);

        // Update global position
        double sinHeading = Math.sin(heading);
        double cosHeading = Math.cos(heading);

        xPos += dForward * cosHeading - dSide * sinHeading;
        yPos += dForward * sinHeading + dSide * cosHeading;
        heading += dHeading;
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

        // Odometry pods (connected to motor ports but no motors attached)
        intake = hardwareMap.get(DcMotor.class, "i");
        intake2 = hardwareMap.get(DcMotor.class, "i2");
        shooter = hardwareMap.get(DcMotor.class, "s");
        flywheel = hardwareMap.get(DcMotorEx.class, "s");
//        shooterHinge = hardwareMap.get(Servo.class, "sH");
        intakeToShooter = hardwareMap.get(CRServo.class, "its");
        intakeToShooter2 = hardwareMap.get(CRServo.class, "its2");


        // --- Odometry encoder setup ---
        telemetry.addData("Before Reset - Left", intake.getCurrentPosition());
        telemetry.addData("Before Reset - Right", intake2.getCurrentPosition());
        telemetry.addData("Before Reset - Back", shooter.getCurrentPosition());
        telemetry.update();
        sleep(1000);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setPower(0);
        intake2.setPower(0);
        shooter.setPower(0);

        prevLeft = intake.getCurrentPosition();
        prevRight = intake2.getCurrentPosition();
        prevBack = shooter.getCurrentPosition();

        telemetry.addData("After Reset - Left", prevLeft);
        telemetry.addData("After Reset - Right", prevRight);
        telemetry.addData("After Reset - Back", prevBack);
        telemetry.update();
        sleep(2000);

        // --- Motor directions ---
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        shooter.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        dashboard = FtcDashboard.getInstance();
        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();
        runtime.reset();

        double nerf = 0.75;

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


//        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lastPosition = shooter.getCurrentPosition();
        dtTimer.reset();

        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();


        while (opModeIsActive()) {
            updateOdometry();
            double batteryVoltage = getBatteryVoltage();

            double Logdrive = -gamepad1.left_stick_y * nerf;
            double LATdrive = -gamepad1.left_stick_x * nerf;
            double Turndrive = -gamepad1.right_stick_x * nerf;

            TelemetryPacket packet = new TelemetryPacket();

//            if (gamepad1.a && robot_centric) {
//                robot_centric = false;
//                field_centric = true;
//                sleep(200);
//                packet.put("field centric testing", field_centric);
//                packet.put("robot centric testing", robot_centric);
//            }
//            else if (gamepad1.y && field_centric) {
//
//                field_centric = false;
//                robot_centric = true;
//                sleep(200);
//                packet.put("robot centric testing", robot_centric);
//                packet.put("field centric testing", field_centric);
//            }

            dashboard.sendTelemetryPacket(packet);


            // --- Wheel brake toggle ---
            if (gamepad1.left_stick_button && gamepad1.right_stick_button && !wheelBreak) {
                wheelBreak = true;
                sleep(200);

                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                frontLeftDrive.setTargetPosition(0);
                frontRightDrive.setTargetPosition(0);
                backLeftDrive.setTargetPosition(0);
                backRightDrive.setTargetPosition(0);

                frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                wheelBreakTargetFL = frontLeftDrive.getCurrentPosition();
                wheelBreakTargetFR = frontRightDrive.getCurrentPosition();
                wheelBreakTargetBL = backLeftDrive.getCurrentPosition();
                wheelBreakTargetBR = backRightDrive.getCurrentPosition();

            } else if (gamepad1.left_stick_button && gamepad1.right_stick_button && wheelBreak) {
                wheelBreak = false;
                sleep(200);
            }

            if(!slow_mode && gamepad1.right_bumper){
                nerf = 0.1;
                slow_mode = true;
            } else if (slow_mode && gamepad1.right_bumper) {
                nerf = 0.75;
                slow_mode = false;


            }


            // --- Wheel brake control ---
            if (wheelBreak) {
                applyWheelBrake(frontLeftDrive, wheelBreakTargetFL);
                applyWheelBrake(frontRightDrive, wheelBreakTargetFR);
                applyWheelBrake(backLeftDrive, wheelBreakTargetBL);
                applyWheelBrake(backRightDrive, wheelBreakTargetBR);

                //put the stay code here


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
                frontLeftDrive.setPower(Logdrive + LATdrive + Turndrive);
                frontRightDrive.setPower(-Logdrive - LATdrive + Turndrive);
            }

            else if (!wheelBreak && field_centric) {

                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                double y = -gamepad1.left_stick_y * nerf; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * nerf;
                double rx = gamepad1.right_stick_x * nerf;

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (gamepad1.start) {
                    imu.resetYaw();
                    // ... other code ...

                    // The odometry system object must be initialized elsewhere in your OpMode, e.g., in the init() method.
                    // Example placeholder:
                    // private OdometrySystem odometrySystem;

                    // else if (!wheelBreak && field_centric) {
                    //     frontLeftDrive.setZeroPowerBehavior(DcMotor.ZerToPowerBehavior.FLOAT);
                    //     frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    //     backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    //     backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    //     frontLeftDrive.setMode(DcData.RunMode.RUN_USING_ENCODER);
                    //     frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    //     backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    //     backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    //     double y = -gamepad1.left_stick_y * nerf; // Remember, Y stick value is reversed
                    //     double x = gamepad1.left_stick_x * nerf;
                    //     double rx = gamepad1.right_stick_x * nerf;

                    //     // This button choice was made so that it is hard to hit on accident,
                    //     // it can be freely changed based on preference.
                    //     // The equivalent button is start on Xbox-style controllers.
                    //     if (gamepad1.start) {
                    //         // Use odometry system's reset method instead of IMU's
                    //         odometrySystem.resetHeading();
                    //     }

                    //     // Get bot heading from the odometry system, ensuring it's in radians
                    //     double botHeading = odometrySystem.getHeading();

                    //     // Rotate the movement direction counter to the bot's rotation
                    //     double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    //     double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    //     rotX = rotX * 1.1; // Counteract imperfect strafing

                    //     // Denominator is the largest motor power (absolute value) or 1
                    //     // This ensures all the powers maintain the same ratio,
                    //     // but only if at least one is out of the range [-1, 1]
                    //     double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

                    //     double frontLeftPower = (rotY + rotX + rx) / denominator;
                    //     double backLeftPower = (rotY - rotX + rx) / denominator;
                    //     double frontRightPower = (rotY - rotX - rx) / denominator;
                    //     double backRightPower = (rotY + rotX - rx) / denominator;

                    //     frontLeftDrive.setPower(frontLeftPower);
                    //     backLeftDrive.setPower(backLeftPower);
                    //     frontRightDrive.setPower(frontRightPower);
                    //     backRightDrive.setPower(backRightPower);
                    // }




                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                frontLeftDrive.setPower(frontLeftPower);
                backLeftDrive.setPower(backLeftPower);
                frontRightDrive.setPower(frontRightPower);
                backRightDrive.setPower(backRightPower);
            }



            // --- Gamepad 2 controls ---
//            handleIntake();
            handleShooter();
//            handleShooterHinge();

            // --- Dashboard telemetry ---
            sendDashboardTelemetry(batteryVoltage);

            telemetry.update();
            sleep(20);
        }
    }

    private void applyWheelBrake(DcMotor motor, int target) {
        int error = target - motor.getCurrentPosition();
        error = Math.max(-maxError, Math.min(maxError, error));
        double power = kP * error;
        power = Math.max(-maxPower, Math.min(maxPower, power));
        motor.setPower(power);
    }


//    private void handleIntake() {
//        if (gamepad2.left_bumper && !intakeActive) {
//            sleep(200);
//            intake.setPower(intake_speed);
//            intake2.setPower(intake_speed);
//            intakeActive = true;
//        } else if (gamepad2.left_bumper && intakeActive) {
//            sleep(200);
//            intake.setPower(0);
//            intake2.setPower(0);
//            intakeActive = false;
//        }
//    }

    private void handleShooter() {
        if (gamepad2.right_trigger > 0.2) {
            double pos = shooter.getCurrentPosition();
            double dt = dtTimer.seconds();
            intakeToShooter.setPower(intakeToShooter_power);
            intakeToShooter2.setPower(intakeToShooter_power);
            // Prevent division by zero on first loop
            if (dt < 0.001) dt = 0.001;
            dtTimer.reset();

            double deltaTicks = pos - lastPosition;
            lastPosition = pos;

            // Use built-in velocity measurement (more stable than manual calculation)
            double velocityTicksPerSec = flywheel.getVelocity();
            currentRPM = (velocityTicksPerSec / TICKS_PER_REV) * 60.0;

            // ======== 2. Compute Feedforward ========
            ff = 0.0;
            if (targetRPM > 20) {
                // kS provides base power to overcome friction
                // kV scales linearly with RPM
                ff = kS + kV * targetRPM;
            }

            // ======== 3. Compute PID ========
            error = targetRPM - currentRPM;

            // Integral with anti-windup clamping
            integral += error * dt;
            integral = Math.max(-integralLimit, Math.min(integralLimit, integral));

            double derivative = (error - lastError) / dt;
            lastError = error;

            pid = kP_F * error + kI * integral + kD * derivative;

            // ======== 4. Combine Feedforward + PID ========
            output = ff + pid;

            // Clamp to motor limits
            output = Math.max(-1.0, Math.min(1.0, output));

            // Command the motor
            shooter.setPower(output);
        }
        else{
            intakeToShooter.setPower(0);
            intakeToShooter2.setPower(0);
            shooter.setPower(0.24);
        }
    }

//    private void handleShooterHinge() {
//        if (gamepad2.a) {
//            sleep(200);
//            shooterUp = !shooterUp;
//            shooterHinge.setPosition(shooterUp ? 1 : 0);
//        }
//    }

    private void sendDashboardTelemetry(double batteryVoltage) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();

        // --- Draw field grid ---
        canvas.setStroke("#404040");
        for (int i = -72; i <= 72; i += 24) {
            canvas.strokeLine(i, -72, i, 72);
            canvas.strokeLine(-72, i, 72, i);
        }

        canvas.setStroke("#FFFFFF");
        canvas.strokeRect(-72, -72, 144, 144);

        // Origin marker
        canvas.setStroke("#FFFF00");
        canvas.strokeLine(-10, 0, 10, 0);
        canvas.strokeLine(0, -10, 0, 10);

        // Robot rectangle
        double robotSize = 18;
        canvas.setStroke("#3FBAFF");
        canvas.setFill("#3FBAFF");
        canvas.fillRect(xPos - robotSize / 2, yPos - robotSize / 2, robotSize, robotSize);

        // Heading indicator
        double headingLineLength = 12;
        double headingX = xPos + headingLineLength * Math.cos(heading);
        double headingY = yPos + headingLineLength * Math.sin(heading);
        canvas.setStroke("#FF0000");
        canvas.setStrokeWidth(3);
        canvas.strokeLine(xPos, yPos, headingX, headingY);

        // Center point
        canvas.setStroke("#00FF00");
        canvas.fillCircle(xPos, yPos, 3);

        // Telemetry values
        packet.put("Wheel Brake Active", wheelBreak);
        packet.put("Intake Active", intakeActive);
        packet.put("Shooter Active", shooterActive);
//        packet.put("Shooter Hinge Position", shooterHinge.getPosition());
        packet.put("Robot X (in)", xPos);
        packet.put("Robot Y (in)", yPos);
        packet.put("Heading (rad)", heading);

        packet.put("Heading (deg)", Math.toDegrees(heading));
        packet.put("Front Left Encoder", frontLeftDrive.getCurrentPosition());
        packet.put("Front Right Encoder", frontRightDrive.getCurrentPosition());
        packet.put("Back Left Encoder", backLeftDrive.getCurrentPosition());
        packet.put("Back Right Encoder", backRightDrive.getCurrentPosition());
        packet.put("Odo Left Raw", -intake.getCurrentPosition());
        packet.put("Odo Right Raw", intake2.getCurrentPosition());
        packet.put("Odo Back Raw", shooter.getCurrentPosition());
        packet.put("Nerf Speed", 0.5);
        packet.put("Slow Mode", slow_mode);
        packet.put("Battery Voltage (V)", batteryVoltage);
        packet.put("GE", true);
        packet.put("ITS", intakeToShooter.getPower());
        packet.put("ITS2", intakeToShooter2.getPower());

        packet.put("Target RPM", targetRPM);
        packet.put("Actual RPM", currentRPM);
        packet.put("Error RPM", error);
        packet.put("Error %", (error / targetRPM) * 100.0);

        // Control outputs
        packet.put("Total Power", output);
        packet.put("FF Power", ff);
        packet.put("PID Power", pid);
        packet.put("Integral", integral);

        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Wheel Brake Active", wheelBreak);
//        telemetry.addData("Intake Active", intakeActive);
//        telemetry.addData("Shooter Active", shooterActive);
//        telemetry.addData("Shooter Hinge Position", shooterHinge.getPosition());
        telemetry.addData("Robot X (in)", xPos);
        telemetry.addData("Robot Y (in)", yPos);
        telemetry.addData("Heading (rad)", heading);

        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
        telemetry.addData("Front Left Encoder", frontLeftDrive.getCurrentPosition());
        telemetry.addData("Front Right Encoder", frontRightDrive.getCurrentPosition());
        telemetry.addData("Back Left Encoder", backLeftDrive.getCurrentPosition());
        telemetry.addData("Back Right Encoder", backRightDrive.getCurrentPosition());
        telemetry.addData("Odo Left Raw", -intake.getCurrentPosition());
        telemetry.addData("Odo Right Raw", intake2.getCurrentPosition());
        telemetry.addData("Odo Back Raw", shooter.getCurrentPosition());
        telemetry.addData("Nerf Speed", 0.5);
        telemetry.addData("Slow Mode", slow_mode);
        telemetry.addData("Battery Voltage (V)", batteryVoltage);
        telemetry.addData("GE", true);
    }
}