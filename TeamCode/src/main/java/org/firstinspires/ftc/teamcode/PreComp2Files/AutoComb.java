//package org.firstinspires.ftc.teamcode.PreComp2Files;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
////import org.firstinspires.ftc.teamcode.ObjectDetectionExamplesTeleop.ObeliskIntakeSystem;
//
///**
// * Autonomous program with simple function calls for all robot actions
// * HuskyLens runs detection during INIT, then stops when autonomous starts
// *
// * Example Usage:
// *   shooter("go");     // Start shooter
// *   shooter("stop");   // Stop shooter
// *   intake("in");      // Deploy intake
// *   intake("out");     // Retract intake
// *   drive(24, 0.5);    // Drive forward 24 inches at 50% power
// */
//@Autonomous(name = "Auto Comb", group = "Auto")
//public class AutoComb extends LinearOpMode {
//
//    // --- Drive motors ---
//    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
//
//    // --- Odometry encoders ---
//    private DcMotor odoleft, odoright, odoperp;
//
//    // --- Shooter components ---
//    private Servo shooterHinge;
//    private CRServo intakeToShooter, intakeToShooter2;
//    private MotorPowerRegulator shooterController;
//
//    // --- Intake servos ---
//    private Servo intake, intake2;
//
//    // --- IMU ---
//    private IMU imu;
//
//    // --- Obelisk Intake System (includes HuskyLens) ---
//    private ObeliskIntakeSystem intakeSystem;
//
//    // --- Constants ---
//    private static final double INTAKE_POSITION_IN = 0.5;
//    private static final double INTAKE_POSITION_OUT = 0;
//    private static final double SHOOTER_HINGE_UP = 1.0;
//    private static final double SHOOTER_HINGE_DOWN = 0;
//    private static final double INTAKE_TO_SHOOTER_POWER = 0.5;
//
//    // Odometry constants
//    private static final double TICKS_PER_INCH = 337.2;
//    private static final double TRACK_WIDTH = 13.5;
//    private static final double BACK_WHEEL_OFFSET = 8;
//    private static final double DRIVE_TICKS_PER_REV = 537.7;
//
//    // Odometry position tracking
//    private double xPos = 0, yPos = 0, heading = 0;
//    private int prevLeft = 0, prevRight = 0, prevBack = 0;
//
//    // Runtime
//    private final ElapsedTime runtime = new ElapsedTime();
//
//    @Override
//    public void runOpMode() {
//        // ========== INITIALIZE HARDWARE ==========
//        initializeHardware();
//
//        telemetry.addData("Status", "Hardware Initialized");
//        telemetry.update();
//
//        // ========== INIT PHASE: RUN HUSKYLENS DETECTION ==========
//        telemetry.addLine("===================");
//        telemetry.addLine("INIT PHASE: HuskyLens Active");
//        telemetry.addLine("Press START when ready");
//        telemetry.addLine("===================");
//        telemetry.update();
//
//        // Loop HuskyLens detection during INIT
//        while (!isStarted() && !isStopRequested()) {
//            // Update the intake system (runs HuskyLens detection)
//            intakeSystem.update();
//
//            // Display telemetry
//            telemetry.addLine("=== HUSKYLENS DETECTING ===");
//            intakeSystem.sendTelemetry(telemetry);
//            telemetry.addLine();
//            telemetry.addData("Status", "Waiting for START...");
//            telemetry.update();
//
//            sleep(50);  // Small delay to prevent overwhelming the system
//        }
//
//        // ========== STOP DETECTION WHEN START IS PRESSED ==========
//        if (opModeIsActive()) {
//            // Stop the intake system detection
//            intakeSystem.stop();
//
//            telemetry.addLine("===================");
//            telemetry.addLine("AUTONOMOUS STARTED");
//            telemetry.addLine("HuskyLens Detection STOPPED");
//            telemetry.addLine("===================");
//            telemetry.update();
//            sleep(500);
//
//            runtime.reset();
//
//            // ========== YOUR AUTONOMOUS ROUTINE GOES HERE ==========
//            // Example autonomous sequence:
//
//            telemetry.addData("Status", "Running Auto Path");
//            telemetry.update();
//
//            // Deploy intake
//            intake("in");
//            sleep(500);
//
//            // Drive forward 24 inches
//            drive(24, 0.5);
//            sleep(500);
//
//            // Start shooter
//            shooterHinge("up");
//            sleep(500);
//            shooter("go");
//            sleep(2000); // Wait for shooter to spin up
//
//            // Feed balls to shooter
//            feedShooter(true);
//            sleep(2000);
//            feedShooter(false);
//
//            // Stop shooter
//            shooter("stop");
//            shooterHinge("down");
//
//            // Strafe right 12 inches
//            strafe(12, 0.5);
//            sleep(500);
//
//            // Turn 90 degrees
//            turn(90, 0.3);
//            sleep(500);
//
//            // Retract intake
//            intake("out");
//
//            telemetry.addData("Status", "Auto Complete");
//            telemetry.update();
//        }
//    }
//
//    // ========================================================================
//    // INITIALIZATION
//    // ========================================================================
//
//    private void initializeHardware() {
//        // Drive motors
//        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontl");
//        frontRightDrive = hardwareMap.get(DcMotor.class, "frontr");
//        backLeftDrive = hardwareMap.get(DcMotor.class, "backl");
//        backRightDrive = hardwareMap.get(DcMotor.class, "backr");
//
//        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
//        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
//        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
//        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
//
//        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // Reset encoders
//        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Odometry encoders
//        odoleft = hardwareMap.get(DcMotor.class, "ol");
//        odoright = hardwareMap.get(DcMotor.class, "or");
//        odoperp = hardwareMap.get(DcMotor.class, "perp");
//
//        odoleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        odoright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        odoperp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        odoleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        odoright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        odoperp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        prevLeft = odoleft.getCurrentPosition();
//        prevRight = odoright.getCurrentPosition();
//        prevBack = odoperp.getCurrentPosition();
//
//        // Shooter components
//        shooterHinge = hardwareMap.get(Servo.class, "sH");
//        intakeToShooter = hardwareMap.get(CRServo.class, "its");
//        intakeToShooter2 = hardwareMap.get(CRServo.class, "its2");
//
//        shooterController = new MotorPowerRegulator(hardwareMap, telemetry, "s");
//        shooterController.setTicksPerRev(112.0);
//        shooterController.setMaxRpmUnderLoad(1400.0);
//        shooterController.setTargetRPM(980.0);
//        shooterController.setAllGains(0.0006785714285714286, 0.06, 0.0004, 0.0002, 0.00005);
//
//        // Intake servos
//        intake = hardwareMap.get(Servo.class, "i");
//        intake2 = hardwareMap.get(Servo.class, "i2");
//
//        // IMU
//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
//        imu.initialize(parameters);
//
//        // ========== INITIALIZE OBELISK INTAKE SYSTEM (includes HuskyLens) ==========
//        intakeSystem = new ObeliskIntakeSystem(hardwareMap);
//
//        // Check if it initialized properly
//        if (!intakeSystem.isInitialized()) {
//            telemetry.addData("ERROR", "Intake system failed to initialize!");
//            telemetry.update();
//        } else {
//            telemetry.addData("HuskyLens", "Initialized Successfully");
//            telemetry.update();
//        }
//
//        // Reset ball counter at start
//        intakeSystem.resetBallCounter();
//    }
//
//    // ========================================================================
//    // SHOOTER FUNCTIONS
//    // ========================================================================
//
//    /**
//     * Control the shooter motor
//     * @param action "go" to start shooter, "stop" to stop shooter
//     */
//    public void shooter(String action) {
//        if (action.equalsIgnoreCase("go")) {
//            // Start shooter at target RPM
//            shooterController.setTargetRPM(980.0);
//
//            // Spin up shooter (wait until at target)
//            ElapsedTime timer = new ElapsedTime();
//            while (opModeIsActive() && !shooterController.isAtTarget(50) && timer.seconds() < 3) {
//                shooterController.loop();
//                telemetry.addData("Shooter", "Spinning up... %.0f RPM", shooterController.getCurrentRPM());
//                telemetry.update();
//                sleep(20);
//            }
//
//            telemetry.addData("Shooter", "Ready!");
//            telemetry.update();
//
//        } else if (action.equalsIgnoreCase("stop")) {
//            // Stop shooter
//            shooterController.setTargetRPM(0);
//            shooterController.loop();
//
//            telemetry.addData("Shooter", "Stopped");
//            telemetry.update();
//        }
//    }
//
//    /**
//     * Control the shooter hinge position
//     * @param position "up" for shooting position, "down" for storage position
//     */
//    public void shooterHinge(String position) {
//        if (position.equalsIgnoreCase("up")) {
//            shooterHinge.setPosition(SHOOTER_HINGE_UP);
//            telemetry.addData("Shooter Hinge", "Up");
//        } else if (position.equalsIgnoreCase("down")) {
//            shooterHinge.setPosition(SHOOTER_HINGE_DOWN);
//            telemetry.addData("Shooter Hinge", "Down");
//        }
//        telemetry.update();
//    }
//
//    /**
//     * Control the feed servos that move balls from intake to shooter
//     * @param running true to run, false to stop
//     */
//    public void feedShooter(boolean running) {
//        if (running) {
//            intakeToShooter.setPower(INTAKE_TO_SHOOTER_POWER);
//            intakeToShooter2.setPower(INTAKE_TO_SHOOTER_POWER);
//            telemetry.addData("Feed", "Running");
//        } else {
//            intakeToShooter.setPower(0);
//            intakeToShooter2.setPower(0);
//            telemetry.addData("Feed", "Stopped");
//        }
//        telemetry.update();
//    }
//
//    // ========================================================================
//    // INTAKE FUNCTIONS
//    // ========================================================================
//
//    /**
//     * Control the intake position
//     * @param action "in" to deploy intake, "out" to retract intake
//     */
//    public void intake(String action) {
//        if (action.equalsIgnoreCase("in")) {
//            intake.setPosition(INTAKE_POSITION_IN);
//            intake2.setPosition(INTAKE_POSITION_IN);
//            telemetry.addData("Intake", "Deployed (In)");
//        } else if (action.equalsIgnoreCase("out")) {
//            intake.setPosition(INTAKE_POSITION_OUT);
//            intake2.setPosition(INTAKE_POSITION_OUT);
//            telemetry.addData("Intake", "Retracted (Out)");
//        }
//        telemetry.update();
//    }
//
//    // ========================================================================
//    // MOVEMENT FUNCTIONS
//    // ========================================================================
//
//    /**
//     * Drive forward/backward a specific distance
//     * @param inches Distance to drive (positive = forward, negative = backward)
//     * @param power Motor power (0.0 to 1.0)
//     */
//    public void drive(double inches, double power) {
//        // Calculate target position
//        int targetTicks = (int)(inches * DRIVE_TICKS_PER_REV / (Math.PI * 4)); // 4" wheel diameter
//
//        // Reset encoders
//        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        // Set target positions
//        frontLeftDrive.setTargetPosition(targetTicks);
//        frontRightDrive.setTargetPosition(targetTicks);
//        backLeftDrive.setTargetPosition(targetTicks);
//        backRightDrive.setTargetPosition(targetTicks);
//
//        // Switch to RUN_TO_POSITION mode
//        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // Set power
//        frontLeftDrive.setPower(Math.abs(power));
//        frontRightDrive.setPower(Math.abs(power));
//        backLeftDrive.setPower(Math.abs(power));
//        backRightDrive.setPower(Math.abs(power));
//
//        // Wait until target reached
//        while (opModeIsActive() &&
//                (frontLeftDrive.isBusy() || frontRightDrive.isBusy() ||
//                        backLeftDrive.isBusy() || backRightDrive.isBusy())) {
//            telemetry.addData("Drive", "Moving %.1f inches", inches);
//            telemetry.addData("Position", "FL:%d FR:%d BL:%d BR:%d",
//                    frontLeftDrive.getCurrentPosition(),
//                    frontRightDrive.getCurrentPosition(),
//                    backLeftDrive.getCurrentPosition(),
//                    backRightDrive.getCurrentPosition());
//            telemetry.update();
//        }
//
//        // Stop motors
//        stopDrive();
//
//        // Switch back to RUN_USING_ENCODER
//        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        telemetry.addData("Drive", "Complete");
//        telemetry.update();
//    }
//
//    /**
//     * Strafe left/right a specific distance
//     * @param inches Distance to strafe (positive = right, negative = left)
//     * @param power Motor power (0.0 to 1.0)
//     */
//    public void strafe(double inches, double power) {
//        // Calculate target position (strafing uses different wheel combination)
//        int targetTicks = (int)(inches * DRIVE_TICKS_PER_REV / (Math.PI * 4));
//
//        // Reset encoders
//        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        // Set target positions (mecanum strafe pattern)
//        frontLeftDrive.setTargetPosition(targetTicks);
//        frontRightDrive.setTargetPosition(-targetTicks);
//        backLeftDrive.setTargetPosition(-targetTicks);
//        backRightDrive.setTargetPosition(targetTicks);
//
//        // Switch to RUN_TO_POSITION mode
//        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // Set power
//        frontLeftDrive.setPower(Math.abs(power));
//        frontRightDrive.setPower(Math.abs(power));
//        backLeftDrive.setPower(Math.abs(power));
//        backRightDrive.setPower(Math.abs(power));
//
//        // Wait until target reached
//        while (opModeIsActive() &&
//                (frontLeftDrive.isBusy() || frontRightDrive.isBusy() ||
//                        backLeftDrive.isBusy() || backRightDrive.isBusy())) {
//            telemetry.addData("Strafe", "Moving %.1f inches", inches);
//            telemetry.update();
//        }
//
//        // Stop motors
//        stopDrive();
//
//        // Switch back to RUN_USING_ENCODER
//        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        telemetry.addData("Strafe", "Complete");
//        telemetry.update();
//    }
//
//    /**
//     * Turn to a specific angle using IMU
//     * @param degrees Degrees to turn (positive = clockwise, negative = counter-clockwise)
//     * @param power Motor power (0.0 to 1.0)
//     */
//    public void turn(double degrees, double power) {
//        // Reset IMU yaw
//        imu.resetYaw();
//
//        double targetAngle = degrees;
//        double currentAngle = 0;
//        double error = targetAngle - currentAngle;
//
//        // Turn until within tolerance
//        while (opModeIsActive() && Math.abs(error) > 2) {
//            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//            error = targetAngle - currentAngle;
//
//            // Normalize error to -180 to 180
//            while (error > 180) error -= 360;
//            while (error < -180) error += 360;
//
//            // Set turn power based on error direction
//            double turnPower = Math.signum(error) * power;
//
//            frontLeftDrive.setPower(turnPower);
//            frontRightDrive.setPower(-turnPower);
//            backLeftDrive.setPower(turnPower);
//            backRightDrive.setPower(-turnPower);
//
//            telemetry.addData("Turn", "Target: %.1f Current: %.1f Error: %.1f",
//                    targetAngle, currentAngle, error);
//            telemetry.update();
//        }
//
//        // Stop motors
//        stopDrive();
//
//        telemetry.addData("Turn", "Complete");
//        telemetry.update();
//    }
//
//    /**
//     * Drive with custom forward, lateral, and turn values
//     * @param forward Forward power (-1.0 to 1.0)
//     * @param lateral Lateral power (-1.0 to 1.0, positive = right)
//     * @param turn Turn power (-1.0 to 1.0, positive = clockwise)
//     * @param duration Duration in seconds
//     */
//    public void driveWithTime(double forward, double lateral, double turn, double duration) {
//        ElapsedTime timer = new ElapsedTime();
//
//        while (opModeIsActive() && timer.seconds() < duration) {
//            double frontLeftPower = forward + lateral + turn;
//            double frontRightPower = forward - lateral - turn;
//            double backLeftPower = forward - lateral + turn;
//            double backRightPower = forward + lateral - turn;
//
//            // Normalize powers if any exceed 1.0
//            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
//                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
//            if (maxPower > 1.0) {
//                frontLeftPower /= maxPower;
//                frontRightPower /= maxPower;
//                backLeftPower /= maxPower;
//                backRightPower /= maxPower;
//            }
//
//            frontLeftDrive.setPower(frontLeftPower);
//            frontRightDrive.setPower(frontRightPower);
//            backLeftDrive.setPower(backLeftPower);
//            backRightDrive.setPower(backRightPower);
//
//            telemetry.addData("Drive Time", "%.1f / %.1f seconds", timer.seconds(), duration);
//            telemetry.update();
//        }
//
//        stopDrive();
//    }
//
//    /**
//     * Stop all drive motors
//     */
//    public void stopDrive() {
//        frontLeftDrive.setPower(0);
//        frontRightDrive.setPower(0);
//        backLeftDrive.setPower(0);
//        backRightDrive.setPower(0);
//    }
//
//    // ========================================================================
//    // UTILITY FUNCTIONS
//    // ========================================================================
//
//    /**
//     * Update odometry position tracking
//     */
//    private void updateOdometry() {
//        int leftPos = -1 * odoleft.getCurrentPosition();
//        int rightPos = odoright.getCurrentPosition();
//        int backPos = odoperp.getCurrentPosition();
//
//        int deltaLeft = leftPos - prevLeft;
//        int deltaRight = rightPos - prevRight;
//        int deltaBack = backPos - prevBack;
//
//        prevLeft = leftPos;
//        prevRight = rightPos;
//        prevBack = backPos;
//
//        double dLeft = deltaLeft / TICKS_PER_INCH;
//        double dRight = deltaRight / TICKS_PER_INCH;
//        double dBack = deltaBack / TICKS_PER_INCH;
//
//        double dHeading = (dRight - dLeft) / TRACK_WIDTH;
//        double dForward = (dLeft + dRight) / 2.0;
//        double dSide = dBack - (dHeading * BACK_WHEEL_OFFSET);
//
//        double sinHeading = Math.sin(heading);
//        double cosHeading = Math.cos(heading);
//
//        xPos += dForward * cosHeading - dSide * sinHeading;
//        yPos += dForward * sinHeading + dSide * cosHeading;
//        heading += dHeading;
//    }
//
//    /**
//     * Get current X position in inches
//     */
//    public double getX() {
//        updateOdometry();
//        return xPos;
//    }
//
//    /**
//     * Get current Y position in inches
//     */
//    public double getY() {
//        updateOdometry();
//        return yPos;
//    }
//
//    /**
//     * Get current heading in radians
//     */
//    public double getHeading() {
//        updateOdometry();
//        return heading;
//    }
//
//    /**
//     * Wait with telemetry updates
//     * @param milliseconds Time to wait in milliseconds
//     */
//    public void pause(int milliseconds) {
//        sleep(milliseconds);
//    }
//}