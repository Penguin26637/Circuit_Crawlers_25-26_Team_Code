package org.firstinspires.ftc.teamcode.Rebuilt.MainPrograms;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * TeleOp that uses the MotorPowerRegulator_New class for shooter control and back motor PID
 * DEBUGGED VERSION - Fixed button debouncing and logic issues
 */
@TeleOp(name = "TeleOp Comb (Debugged)", group = "Examples")
@Config
public class teleOpCOMB_New extends LinearOpMode {

    // --- Gamepad 1 drive motors ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // ========== BACK MOTOR CONTROLLERS - USING MotorPowerRegulator_New ==========
    private MotorPowerRegulator_New backLeftController;
    private MotorPowerRegulator_New backRightController;

    // --- Wheel brake ---
    public static boolean wheelBreak = false;
    public static int wheelBreakTargetFL, wheelBreakTargetFR, wheelBreakTargetBL, wheelBreakTargetBR;
    public static double wheelBreak_kP = 0.01;
    public static double wheelBreak_maxPower = 0.2;
    public static int wheelBreak_maxError = 100;

    // --- Odometry encoders (no motors attached, just encoder readings) ---
    private DcMotor odoleft, odoright, odoperp;
    private Servo shooterHinge;
//    private CRServo intakeToShooter, intakeToShooter2;
    private Servo intake, intake2;

    // ========== SHOOTER CONTROLLER - USING MotorPowerRegulator_New CLASS ==========
    private MotorPowerRegulator_New shooterController;

    public static boolean intakeIn = false;
    public static boolean shooterActive = false;
    public static boolean shooterUp = false;

    public static double intake_position_in = 0.5;
    public static double intake_position_out = 0;

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
    public static boolean use_back_motor_pid = false;  // Toggle for back motor PID control
    public static double kP = 0.01;      // Proportional control constant
    public static double maxPower = 0.2; // Maximum motor power (range: 0–1)
    public static int maxError = 100;

    public static double nerf = 0.75;

    public static double idlePower = 0.25;

    // Motor encoder constants for drive motors
    public static double DRIVE_TICKS_PER_REV = 537.7;  // Adjust for your drive motors

    // --- Button debouncing variables ---
    private boolean lastGamepad1B = false;
    private boolean lastGamepad1A = false;
    private boolean lastGamepad1RightBumper = false;
    private boolean lastGamepad2LeftBumper = false;
    private boolean lastGamepad2A = false;
    private boolean lastWheelBreakButtons = false;

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
        int leftPos = -1*(odoleft.getCurrentPosition());
        int rightPos = (odoright.getCurrentPosition());
        int backPos = (odoperp.getCurrentPosition());

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

    // The intake decision system
    private ObeliskIntakeSystem_New intakeSystem;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
//        telemetry.update();

        // Initialize your existing hardware
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backl");
        backRightDrive = hardwareMap.get(DcMotor.class, "backr");

        odoleft = hardwareMap.get(DcMotor.class, "ol");
        odoright = hardwareMap.get(DcMotor.class, "or");
        odoperp = hardwareMap.get(DcMotor.class, "perp");
//        shooterHinge = hardwareMap.get(Servo.class, "sH");
//        intakeToShooter = hardwareMap.get(CRServo.class, "its");
//        intakeToShooter2 = hardwareMap.get(CRServo.class, "its2");
        intake = hardwareMap.get(Servo.class, "i");
        intake2 = hardwareMap.get(Servo.class, "i2");

        // ========== INITIALIZE SHOOTER CONTROLLER ==========
        shooterController = new MotorPowerRegulator_New(hardwareMap, telemetry, "s");

        // Configure shooter parameters
        shooterController.setTicksPerRev(112.0);
        shooterController.setMaxRpmUnderLoad(1400.0);
        shooterController.setTargetRPM(980.0);
        shooterController.setAllGains(0.0006785714285714286, 0.06, 0.0004, 0.0002, 0.00005);

        // ========== INITIALIZE BACK MOTOR CONTROLLERS =========================
        backLeftController = new MotorPowerRegulator_New(hardwareMap, telemetry, "backl");
        backRightController = new MotorPowerRegulator_New(hardwareMap, telemetry, "backr");

        // Configure back motors
        backLeftController.setTicksPerRev(DRIVE_TICKS_PER_REV);
        backLeftController.setMaxRpmUnderLoad(300.0);  // Adjust based on your drive motors
        backLeftController.setAllGains(0.00068, 0.06, 0.0004, 0.0002, 0.00005);

        backRightController.setTicksPerRev(DRIVE_TICKS_PER_REV);
        backRightController.setMaxRpmUnderLoad(300.0);
        backRightController.setAllGains(0.00068, 0.06, 0.0004, 0.0002, 0.00005);

        // ========== INITIALIZE INTAKE SYSTEM ==========
        intakeSystem = new ObeliskIntakeSystem_New(hardwareMap);

        // Check if it initialized properly
        if (!intakeSystem.isInitialized()) {
            telemetry.addData("ERROR", "Intake system failed to initialize!");
//            telemetry.update();
        }

        // Reset ball counter at start
        intakeSystem.resetBallCounter();

        // --- Odometry encoder setup ---
        telemetry.addData("Before Reset - Left", odoleft.getCurrentPosition());
        telemetry.addData("Before Reset - Right", odoright.getCurrentPosition());
        telemetry.addData("Before Reset - Back", odoperp.getCurrentPosition());
//        telemetry.update();
        sleep(1000);

        odoleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoperp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);

        odoleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoperp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        odoleft.setPower(0);
        odoright.setPower(0);
        odoperp.setPower(0);

        prevLeft = odoleft.getCurrentPosition();
        prevRight = odoright.getCurrentPosition();
        prevBack = odoperp.getCurrentPosition();

        telemetry.addData("After Reset - Left", prevLeft);
        telemetry.addData("After Reset - Right", prevRight);
        telemetry.addData("After Reset - Back", prevBack);
//        telemetry.update();
        sleep(2000);

        // --- Motor directions ---
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        dashboard = FtcDashboard.getInstance();
        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        while (opModeIsActive()) {

            // ========== YOUR EXISTING DRIVE CODE ==========
            updateOdometry();
            double batteryVoltage = getBatteryVoltage();

            double Logdrive = -gamepad1.left_stick_y * nerf;
            double LATdrive = -gamepad1.left_stick_x * nerf;
            double Turndrive = -gamepad1.right_stick_x * nerf;

            // Determine if turning
            boolean turning = Math.abs(Turndrive) > 0.1;

            TelemetryPacket packet = new TelemetryPacket();

            // ========== FIXED: Toggle back motor PID mode with gamepad1.b (proper debouncing) ==========
            if (gamepad1.b && !lastGamepad1B) {
                use_back_motor_pid = !use_back_motor_pid;
                telemetry.addLine(use_back_motor_pid ? "Back Motor PID: ON" : "Back Motor PID: OFF");
            }

            lastGamepad1B = gamepad1.b;

            // ========== FIXED: Toggle drive mode with gamepad1.a (proper debouncing) ==========
            if (gamepad1.a && !lastGamepad1A) {
                if (robot_centric) {
                    robot_centric = false;
                    field_centric = true;
                    telemetry.addData("Robot centric active", robot_centric);
                    telemetry.addData("Field centric active", field_centric);
                } else if (field_centric) {
                    field_centric = false;
                    robot_centric = true;
                    telemetry.addData("Robot centric active", robot_centric);
                    telemetry.addData("Field centric active", field_centric);
                }
            }

            lastGamepad1A = gamepad1.a;

            packet.put("robot centric testing", robot_centric);
            packet.put("field centric testing", field_centric);
            dashboard.sendTelemetryPacket(packet);

            // ========== FIXED: Wheel brake toggle (proper debouncing) ==========
            boolean currentWheelBreakButtons = gamepad1.left_stick_button && gamepad1.right_stick_button;

            if (currentWheelBreakButtons && !lastWheelBreakButtons) {
                wheelBreak = !wheelBreak;

                if (wheelBreak) {
                    // Entering wheel brake mode
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
                }
            }
            lastWheelBreakButtons = currentWheelBreakButtons;

            // ========== FIXED: Slow mode toggle (proper debouncing) ==========
            if (gamepad1.right_bumper && !lastGamepad1RightBumper) {
                slow_mode = !slow_mode;
                nerf = slow_mode ? 0.1 : 0.75;
            }
            lastGamepad1RightBumper = gamepad1.right_bumper;

            // ========== DRIVE MODES ==========
            if (wheelBreak) {
                applyWheelBrake(frontLeftDrive, wheelBreakTargetFL);
                applyWheelBrake(frontRightDrive, wheelBreakTargetFR);
                applyWheelBrake(backLeftDrive, wheelBreakTargetBL);
                applyWheelBrake(backRightDrive, wheelBreakTargetBR);

                telemetry.addData("WHEEL BRAKE ACTIVE", "True");
            }
            // ========== BACK MOTOR PID MODE ==========
            else if (!wheelBreak && robot_centric) {
                // Set modes for PID control
                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backLeftController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backRightController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeftController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backRightController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                // Get RPM from front motors
                DcMotorEx frontLeftEx = (DcMotorEx) frontLeftDrive;
                DcMotorEx frontRightEx = (DcMotorEx) frontRightDrive;

                double frontLeftVelocity = frontLeftEx.getVelocity();
                double frontRightVelocity = frontRightEx.getVelocity();

                double frontLeftRPM = (frontLeftVelocity / DRIVE_TICKS_PER_REV) * 60.0;
                double frontRightRPM = (frontRightVelocity / DRIVE_TICKS_PER_REV) * 60.0;

                // Set front motor powers
                frontLeftDrive.setPower(Logdrive + LATdrive + Turndrive);
                frontRightDrive.setPower(-Logdrive - LATdrive + Turndrive);

                // Match back motors to front motors with PID
                if (!turning) {
                    // Straight driving
                    backLeftController.setTargetRPM(Math.abs(frontRightRPM));
                    backRightController.setTargetRPM(Math.abs(frontLeftRPM));
                } else {
                    // Turning
                    backLeftController.setTargetRPM(Math.abs(frontLeftRPM));
                    backRightController.setTargetRPM(Math.abs(frontRightRPM));
                }

                // Update PID controllers
                backLeftController.loop();
                backRightController.loop();
            }
            // ========== NORMAL DRIVE MODE ==========
//            else if (!wheelBreak && robot_centric) {
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
//                backLeftDrive.setPower(Logdrive - LATdrive + Turndrive);
//                backRightDrive.setPower(-Logdrive + LATdrive + Turndrive);
//                frontLeftDrive.setPower(Logdrive + LATdrive + Turndrive);
//                frontRightDrive.setPower(-Logdrive - LATdrive + Turndrive);
//            }
            // ========== FIELD CENTRIC MODE ==========
            else if (!wheelBreak && field_centric) {

                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                double y = -gamepad1.left_stick_y * nerf;
                double x = gamepad1.left_stick_x * nerf;
                double rx = gamepad1.right_stick_x * nerf;

                if (gamepad1.start) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                frontLeftDrive.setPower(frontLeftPower * nerf);
                backLeftDrive.setPower(backLeftPower * nerf);
                frontRightDrive.setPower(frontRightPower * nerf);
                backRightDrive.setPower(backRightPower * nerf);
            }

            handleIntake();
            handleShooter();
//            handleShooterHinge();

            // ========== OBELISK INTAKE SYSTEM - JUST TWO LINES! ==========
            // Update the system (detects obelisk and balls)
            intakeSystem.update();

            // --- Dashboard telemetry ---
//            sendDashboardTelemetry(batteryVoltage);

            // Intake system telemetry (just one line!)
//            intakeSystem.sendTelemetry(telemetry);

//            telemetry.update();
            sleep(20);
            telemetry.addLine("=== BACK MOTORS ===");
            telemetry.addData("Left Target", backLeftController.getTargetRPM());
            telemetry.addData("Left Actual", backLeftController.getCurrentRPM());
            telemetry.addData("Right Target", backRightController.getTargetRPM());
            telemetry.addData("Right Actual", backRightController.getCurrentRPM());
        }

        // Cleanup
        intakeSystem.stop();
    }

    private void applyWheelBrake(DcMotor motor, int target) {
        int error = target - motor.getCurrentPosition();
        error = Math.max(-maxError, Math.min(maxError, error));
        double power = kP * error;
        power = Math.max(-maxPower, Math.min(maxPower, power));
        motor.setPower(power);
    }

    // ========== FIXED: Proper button debouncing for intake toggle ==========
    private void handleIntake() {
        if (gamepad2.left_bumper && !lastGamepad2LeftBumper) {
            intakeIn = !intakeIn;
            intake.setPosition(intakeIn ? intake_position_in : intake_position_out);
            intake2.setPosition(intakeIn ? intake_position_in : intake_position_out);
        }
        lastGamepad2LeftBumper = gamepad2.left_bumper;
    }

    private void handleShooter() {
        // ========== USING MotorPowerRegulator_New CLASS ==========
        telemetry.addData("Right Trigger", gamepad2.right_trigger);
        if (gamepad2.right_trigger >= 0.2) {
            telemetry.addData("Actual RPM", shooterController.getCurrentRPM());
            telemetry.addData("Target RPM", shooterController.getTargetRPM());
            // Run intake-to-shooter servos
//            intakeToShooter.setPower(intakeToShooter_power);
//            intakeToShooter2.setPower(intakeToShooter_power);

            // Update the shooter controller (handles all PID+FF logic)
//            shooterController.loop();
            telemetry.addLine("Here");
            shooterController.setTargetRPM(1400);
            shooterController.loop();
            shooterActive = true;

        } else {
            telemetry.addLine("Failed");
            telemetry.addData("Actual RPM", shooterController.getCurrentRPM());
            telemetry.addData("Target RPM", shooterController.getTargetRPM());
            // Stop intake-to-shooter servos
//            intakeToShooter.setPower(0);
//            intakeToShooter2.setPower(0);

            // Set shooter to idle
            shooterController.stop();
//            shooterController.setTargetRPM(500);
//            shooterController.loop();

            shooterActive = false;
        }
    }

    // ========== FIXED: Proper button debouncing for shooter hinge toggle ==========
//    private void handleShooterHinge() {
//        if (gamepad2.a && !lastGamepad2A) {
//            shooterUp = !shooterUp;
////            shooterHinge.setPosition(shooterUp ? 1 : 0);
//        }
//        lastGamepad2A = gamepad2.a;
//    }

//    private void sendDashboardTelemetry(double batteryVoltage)
//        TelemetryPacket packet = new TelemetryPacket();
//        Canvas canvas = packet.fieldOverlay();
//
//        // --- Draw field grid ---
//        canvas.setStroke("#404040");
//        for (int i = -72; i <= 72; i += 24) {
//            canvas.strokeLine(i, -72, i, 72);
//            canvas.strokeLine(-72, i, 72, i);
//        }
//
//        canvas.setStroke("#FFFFFF");
//        canvas.strokeRect(-72, -72, 144, 144);
//
//        // Origin marker
//        canvas.setStroke("#FFFF00");
//        canvas.strokeLine(-10, 0, 10, 0);
//        canvas.strokeLine(0, -10, 0, 10);
//
//        // Robot rectangle
//        double robotSize = 18;
//        canvas.setStroke("#3FBAFF");
//        canvas.setFill("#3FBAFF");
//        canvas.fillRect(xPos - robotSize / 2, yPos - robotSize / 2, robotSize, robotSize);
//
//        // Heading indicator
//        double headingLineLength = 12;
//        double headingX = xPos + headingLineLength * Math.cos(heading);
//        double headingY = yPos + headingLineLength * Math.sin(heading);
//        canvas.setStroke("#FF0000");
//        canvas.setStrokeWidth(3);
//        canvas.strokeLine(xPos, yPos, headingX, headingY);
//
//        // Center point
//        canvas.setStroke("#00FF00");
//        canvas.fillCircle(xPos, yPos, 3);

        // ========== TELEMETRY ==========
//        packet.put("Wheel Brake Active", wheelBreak);
//        packet.put("Back Motor PID Mode", use_back_motor_pid);
//        packet.put("Shooter Active", shooterActive);
//        packet.put("Shooter Hinge Position", shooterHinge.getPosition());
//        packet.put("Robot X (in)", xPos);
//        packet.put("Robot Y (in)", yPos);
//        packet.put("Heading (rad)", heading);
//        packet.put("Heading (deg)", Math.toDegrees(heading));
//        packet.put("Nerf Speed", nerf);
//        packet.put("Slow Mode", slow_mode);
//        packet.put("Battery Voltage (V)", batteryVoltage);
//
//        // Shooter data
//        packet.put("Shooter Target RPM", shooterController.getTargetRPM());
//        packet.put("Shooter Actual RPM", shooterController.getCurrentRPM());
//        packet.put("Shooter At Target", shooterController.isAtTarget(50));

        // Back motor data (if PID mode enabled)
//        if (use_back_motor_pid) {
//            packet.put("Back Left Target RPM", backLeftController.getTargetRPM());
//            packet.put("Back Left Actual RPM", backLeftController.getCurrentRPM());
//            packet.put("Back Right Target RPM", backRightController.getTargetRPM());
//            packet.put("Back Right Actual RPM", backRightController.getCurrentRPM());
//        }
//
//        packet.put("robot centric", robot_centric);
//        packet.put("field centric", field_centric);
//        packet.put("intake in", intakeIn);
//        packet.put("shooter up", shooterUp);

//

        // Driver Station telemetry
//        telemetry.addData("Wheel Brake Active", wheelBreak);
//        telemetry.addData("Back Motor PID", use_back_motor_pid ? "ON" : "OFF");
//        telemetry.addData("Shooter Active", shooterActive);
//        telemetry.addData("Target RPM", shooterController.getTargetRPM());
//        telemetry.addData("Actual RPM", shooterController.getCurrentRPM());
//        telemetry.addData("At Target?", shooterController.isAtTarget(50) ? "✓" : "✗");
//        telemetry.addData("Battery (V)", batteryVoltage);
//        telemetry.addData("Robot Centric", robot_centric);
//        telemetry.addData("Field Centric", field_centric);
//
//        if (use_back_motor_pid) {
//          telemetry.addLine();
//        telemetry.addLine("=== BACK MOTORS ===");
//        telemetry.addData("Left Target", backLeftController.getTargetRPM());
//        telemetry.addData("Left Actual", backLeftController.getCurrentRPM());
//        telemetry.addData("Right Target", backRightController.getTargetRPM());
//        telemetry.addData("Right Actual", backRightController.getCurrentRPM());

//
//        telemetry.addData("")
}
