package org.firstinspires.ftc.teamcode.PreComp2Files;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.ObjectDetectionExamplesTeleop.ObeliskIntakeSystem;


/**
 * TeleOp with MotorPowerRegulator for shooter and back motor control
 * Features: Robot/Field centric drive, wheel brake, PID motor control, intake system
 */
@TeleOp(name = "TeleOp Comb (Debugged)", group = "Examples")
@Config
@Disabled
public class Testing extends LinearOpMode {

    // === HARDWARE ===
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotor odoleft, odoright, odoperp;
    private Servo intake, intake2;
    private IMU imu;

    // === MOTOR CONTROLLERS ===
    private MotorPowerRegulator shooterController;
    private MotorPowerRegulator backLeftController;
    private MotorPowerRegulator backRightController;

    // === INTAKE SYSTEM ===
    private ObeliskIntakeSystem intakeSystem;

    // === CONFIGURATION - Wheel Brake ===
    public static boolean wheelBreak = false;
    public static int wheelBreakTargetFL, wheelBreakTargetFR, wheelBreakTargetBL, wheelBreakTargetBR;
    public static double wheelBreak_kP = 0.01;
    public static double wheelBreak_maxPower = 0.2;
    public static int wheelBreak_maxError = 100;

    // === CONFIGURATION - Drive ===
    public static boolean slow_mode = false;
    public static boolean robot_centric = true;
    public static boolean field_centric = false;
    public static double nerf = 0.75;
    public static double DRIVE_TICKS_PER_REV = 537.7;

    // === CONFIGURATION - Intake ===
    public static boolean intakeIn = false;
    public static double intake_position_in = 0.5;
    public static double intake_position_out = 0;

    // === CONFIGURATION - Shooter ===
    public static boolean shooterActive = false;
    public static boolean shooterUp = false;

    // === CONFIGURATION - Odometry ===
    public static double TICKS_PER_INCH = 337.2;
    public static double TRACK_WIDTH = 13.5;
    public static double BACK_WHEEL_OFFSET = 8;

    // === STATE ===
    private double xPos = 0, yPos = 0, heading = 0;
    private int prevLeft = 0, prevRight = 0, prevBack = 0;
    private final ElapsedTime runtime = new ElapsedTime();
    private FtcDashboard dashboard;

    // === BUTTON DEBOUNCING ===
    private boolean lastGamepad1B = false;
    private boolean lastGamepad1A = false;
    private boolean lastGamepad1RightBumper = false;
    private boolean lastGamepad2LeftBumper = false;
    private boolean lastGamepad2A = false;
    private boolean lastWheelBreakButtons = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        initializeHardware();
        initializeMotorControllers();
        initializeIntakeSystem();
        initializeOdometry();
        configureDriveMotors();

        dashboard = FtcDashboard.getInstance();
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();
        runtime.reset();

        initializeIMU();

        while (opModeIsActive()) {
            updateOdometry();

            double batteryVoltage = getBatteryVoltage();
            double Logdrive = -gamepad1.left_stick_y * nerf;
            double LATdrive = -gamepad1.left_stick_x * nerf;
            double Turndrive = -gamepad1.right_stick_x * nerf;
            boolean turning = Math.abs(Turndrive) > 0.1;

            handleGamepad1Controls();
            handleDriveMode(Logdrive, LATdrive, Turndrive, turning);
            handleIntake();
            handleShooter();

            intakeSystem.update();
            intakeSystem.sendTelemetry(telemetry);

            sendDashboardTelemetry(batteryVoltage);
            telemetry.update();

            sleep(20);
        }

        intakeSystem.stop();
    }

    // === INITIALIZATION METHODS ===

    private void initializeHardware() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backl");
        backRightDrive = hardwareMap.get(DcMotor.class, "backr");

        odoleft = hardwareMap.get(DcMotor.class, "ol");
        odoright = hardwareMap.get(DcMotor.class, "or");
        odoperp = hardwareMap.get(DcMotor.class, "perp");

        intake = hardwareMap.get(Servo.class, "i");
        intake2 = hardwareMap.get(Servo.class, "i2");
    }

    private void initializeMotorControllers() {
        // Shooter Controller
        shooterController = new MotorPowerRegulator(hardwareMap, telemetry, "s");
        shooterController.setTicksPerRev(112.0);
        shooterController.setMaxRpmUnderLoad(1400.0);
        shooterController.setTargetRPM(980.0);
        shooterController.setAllGains(0.0006785714285714286, 0.06, 0.0004, 0.0002, 0.00005);

        // Back Motor Controllers
        backLeftController = new MotorPowerRegulator(hardwareMap, telemetry, "backl");
        backLeftController.setTicksPerRev(DRIVE_TICKS_PER_REV);
        backLeftController.setMaxRpmUnderLoad(300.0);
        backLeftController.setAllGains(0.00068, 0.06, 0.0004, 0.0002, 0.00005);

        backRightController = new MotorPowerRegulator(hardwareMap, telemetry, "backr");
        backRightController.setTicksPerRev(DRIVE_TICKS_PER_REV);
        backRightController.setMaxRpmUnderLoad(300.0);
        backRightController.setAllGains(0.00068, 0.06, 0.0004, 0.0002, 0.00005);
    }

    private void initializeIntakeSystem() {
        intakeSystem = new ObeliskIntakeSystem(hardwareMap);

        if (!intakeSystem.isInitialized()) {
            telemetry.addData("ERROR", "Intake system failed to initialize!");
            telemetry.update();
        }

        intakeSystem.resetBallCounter();
    }

    private void initializeOdometry() {
        telemetry.addData("Before Reset - Left", odoleft.getCurrentPosition());
        telemetry.addData("Before Reset - Right", odoright.getCurrentPosition());
        telemetry.addData("Before Reset - Back", odoperp.getCurrentPosition());
        telemetry.update();
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
        telemetry.update();
        sleep(2000);
    }

    private void configureDriveMotors() {
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void initializeIMU() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
    }

    // === UTILITY METHODS ===

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    private void updateOdometry() {
        int leftPos = -1 * (odoleft.getCurrentPosition());
        int rightPos = (odoright.getCurrentPosition());
        int backPos = (odoperp.getCurrentPosition());

        int deltaLeft = leftPos - prevLeft;
        int deltaRight = rightPos - prevRight;
        int deltaBack = backPos - prevBack;

        prevLeft = leftPos;
        prevRight = rightPos;
        prevBack = backPos;

        double dLeft = deltaLeft / TICKS_PER_INCH;
        double dRight = deltaRight / TICKS_PER_INCH;
        double dBack = deltaBack / TICKS_PER_INCH;

        double dHeading = (dRight - dLeft) / TRACK_WIDTH;
        double dForward = (dLeft + dRight) / 2.0;
        double dSide = dBack - (dHeading * BACK_WHEEL_OFFSET);

        double sinHeading = Math.sin(heading);
        double cosHeading = Math.cos(heading);

        xPos += dForward * cosHeading - dSide * sinHeading;
        yPos += dForward * sinHeading + dSide * cosHeading;
        heading += dHeading;
    }

    // === CONTROL HANDLERS ===

    private void handleGamepad1Controls() {
        // Toggle drive mode (robot centric <-> field centric)
        if (gamepad1.a && !lastGamepad1A) {
            if (robot_centric) {
                robot_centric = false;
                field_centric = true;
            } else {
                field_centric = false;
                robot_centric = true;
            }
        }
        lastGamepad1A = gamepad1.a;

        // Toggle slow mode
        if (gamepad1.right_bumper && !lastGamepad1RightBumper) {
            slow_mode = !slow_mode;
            nerf = slow_mode ? 0.1 : 0.75;
        }
        lastGamepad1RightBumper = gamepad1.right_bumper;

        // Toggle wheel brake
        boolean currentWheelBreakButtons = gamepad1.left_stick_button && gamepad1.right_stick_button;
        if (currentWheelBreakButtons && !lastWheelBreakButtons) {
            wheelBreak = !wheelBreak;

            if (wheelBreak) {
                enableWheelBrake();
            }
        }
        lastWheelBreakButtons = currentWheelBreakButtons;

        // Reset IMU heading (for field centric)
        if (gamepad1.start) {
            imu.resetYaw();
        }
    }

    private void enableWheelBrake() {
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

    private void handleDriveMode(double Logdrive, double LATdrive, double Turndrive, boolean turning) {
        if (wheelBreak) {
            applyWheelBrake(frontLeftDrive, wheelBreakTargetFL);
            applyWheelBrake(frontRightDrive, wheelBreakTargetFR);
            applyWheelBrake(backLeftDrive, wheelBreakTargetBL);
            applyWheelBrake(backRightDrive, wheelBreakTargetBR);
        } else if (robot_centric) {
            handleRobotCentricDrive(Logdrive, LATdrive, Turndrive, turning);
        } else if (field_centric) {
            handleFieldCentricDrive();
        }
    }

    private void handleRobotCentricDrive(double Logdrive, double LATdrive, double Turndrive, boolean turning) {
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
            backLeftController.setTargetRPM(Math.abs(frontRightRPM));
            backRightController.setTargetRPM(Math.abs(frontLeftRPM));
        } else {
            backLeftController.setTargetRPM(Math.abs(frontLeftRPM));
            backRightController.setTargetRPM(Math.abs(frontRightRPM));
        }

        backLeftController.loop();
        backRightController.loop();
    }

    private void handleFieldCentricDrive() {
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

    private void applyWheelBrake(DcMotor motor, int target) {
        int error = target - motor.getCurrentPosition();
        error = Math.max(-wheelBreak_maxError, Math.min(wheelBreak_maxError, error));
        double power = wheelBreak_kP * error;
        power = Math.max(-wheelBreak_maxPower, Math.min(wheelBreak_maxPower, power));
        motor.setPower(power);
    }

    private void handleIntake() {
        if (gamepad2.left_bumper && !lastGamepad2LeftBumper) {
            intakeIn = !intakeIn;
            intake.setPosition(intakeIn ? intake_position_in : intake_position_out);
            intake2.setPosition(intakeIn ? intake_position_in : intake_position_out);
        }
        lastGamepad2LeftBumper = gamepad2.left_bumper;
    }

    private void handleShooter() {
        if (gamepad2.right_trigger > 0.2) {
            shooterController.loop();
            shooterActive = true;
        } else {
            shooterController.setTargetRPM(0.24);
            shooterController.loop();
            shooterActive = false;
        }
    }

    // === TELEMETRY ===

    private void sendDashboardTelemetry(double batteryVoltage) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();

        drawFieldGrid(canvas);
        drawRobot(canvas);

        // Dashboard data
        packet.put("Wheel Brake Active", wheelBreak);
        packet.put("Shooter Active", shooterActive);
        packet.put("Robot X (in)", xPos);
        packet.put("Robot Y (in)", yPos);
        packet.put("Heading (rad)", heading);
        packet.put("Heading (deg)", Math.toDegrees(heading));
        packet.put("Nerf Speed", nerf);
        packet.put("Slow Mode", slow_mode);
        packet.put("Battery Voltage (V)", batteryVoltage);
        packet.put("Shooter Target RPM", shooterController.getTargetRPM());
        packet.put("Shooter Actual RPM", shooterController.getCurrentRPM());
        packet.put("Shooter At Target", shooterController.isAtTarget(50));
        packet.put("Back Left Target RPM", backLeftController.getTargetRPM());
        packet.put("Back Left Actual RPM", backLeftController.getCurrentRPM());
        packet.put("Back Right Target RPM", backRightController.getTargetRPM());
        packet.put("Back Right Actual RPM", backRightController.getCurrentRPM());
        packet.put("Robot Centric", robot_centric);
        packet.put("Field Centric", field_centric);
        packet.put("Intake In", intakeIn);

        // Driver Station telemetry
        telemetry.addData("Wheel Brake Active", wheelBreak);
        telemetry.addData("Shooter Active", shooterActive);
        telemetry.addData("Target RPM", shooterController.getTargetRPM());
        telemetry.addData("Actual RPM", shooterController.getCurrentRPM());
        telemetry.addData("At Target?", shooterController.isAtTarget(50) ? "✓" : "✗");
        telemetry.addData("Battery (V)", batteryVoltage);
        telemetry.addData("Robot Centric", robot_centric);
        telemetry.addData("Field Centric", field_centric);
        telemetry.addLine();
        telemetry.addLine("=== BACK MOTORS ===");
        telemetry.addData("Left Target", backLeftController.getTargetRPM());
        telemetry.addData("Left Actual", backLeftController.getCurrentRPM());
        telemetry.addData("Right Target", backRightController.getTargetRPM());
        telemetry.addData("Right Actual", backRightController.getCurrentRPM());

        dashboard.sendTelemetryPacket(packet);
    }

    private void drawFieldGrid(Canvas canvas) {
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
    }

    private void drawRobot(Canvas canvas) {
        double robotSize = 18;

        // Robot rectangle
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
    }
}