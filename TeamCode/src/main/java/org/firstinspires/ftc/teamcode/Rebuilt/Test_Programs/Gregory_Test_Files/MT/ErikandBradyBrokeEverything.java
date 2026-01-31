package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.MT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Rebuilt.MainPrograms.MotorPowerRegulator_New;

@TeleOp(name = "ErikandBradyBrokeEverything", group = "Main")
public class ErikandBradyBrokeEverything extends LinearOpMode {

    // Hardware
    private DriveControlClassGreg drive;
    private DcMotor odoLeft, odoRight, odoPerp;
    private ServoClass intakeLeft, intakeRight, spindexer, flipper;
    private MotorPowerRegulator_New shooter;
    private MultipleTelemetry multitelemetry;

    // Configuration
    private static final double NORMAL_SPEED = 0.75, SLOW_SPEED = 0.1;
    private static final boolean ENABLE_ODOMETRY = true;
    private static final double SHOOTER_ACTIVE_RPM = 1400.0, SHOOTER_IDLE_RPM = 700.0;
    private static final double TICKS_PER_INCH = 337.2, TRACK_WIDTH = 13.5, PERP_OFFSET = 8.0;

    // State
    private double xPos = 0, yPos = 0, odoHeading = 0;
    private int prevLeft = 0, prevRight = 0, prevPerp = 0;
    private boolean odometryInitialized = false;
    private boolean lastGamepad1A = false, lastGamepad1X = false, lastGamepad1RightBumper = false;
    private boolean lastGamepad1Start = false, lastGamepad1Back = false;

    @Override
    public void runOpMode() {
        multitelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        multitelemetry.addData("Status", "Initializing...");
        multitelemetry.update();

        try {
            // Initialize drive
            drive = new DriveControlClassGreg(hardwareMap, multitelemetry, true, true, false);
            drive.nerf = NORMAL_SPEED;
            drive.useFieldCentric = false;
            drive.useImuForFieldCentric = true;

            // Initialize servos
            intakeLeft = new ServoClass("intakeLeft", ServoClass.ServoType.CONTINUOUS_SERVO);
            intakeRight = new ServoClass("intakeRight", ServoClass.ServoType.CONTINUOUS_SERVO);
            spindexer = new ServoClass("spindexer", ServoClass.ServoType.CONTINUOUS_SERVO);
            flipper = new ServoClass("flipper", ServoClass.ServoType.STANDARD_SERVO);

            intakeLeft.init(hardwareMap);
            intakeRight.init(hardwareMap);
            spindexer.init(hardwareMap);
            flipper.init(hardwareMap);

            intakeLeft.stop();
            intakeRight.stop();
            spindexer.stop();
            flipper.stop();

            // Initialize shooter
            shooter = new MotorPowerRegulator_New(hardwareMap, multitelemetry, "shooter");
            shooter.setTicksPerRev(537.7);
            shooter.setMaxRpmUnderLoad(300.0);
            shooter.setAllGains(0.00068, 0.06, 0.0004, 0.0002, 0.00005);
            shooter.setTargetRPM(SHOOTER_IDLE_RPM);

            multitelemetry.addData("Status", "Initialized!");
            multitelemetry.addData("Drive Motors", drive.driveMotorsInitialized ? "✓" : "✗");
            multitelemetry.addData("IMU", drive.imuInitialized ? "✓" : "✗");
            multitelemetry.addData("Intake L/R", (intakeLeft.getCRServo() != null ? "✓" : "✗") + "/" + (intakeRight.getCRServo() != null ? "✓" : "✗"));
            multitelemetry.addData("Spindexer", spindexer.getCRServo() != null ? "✓" : "✗");
            multitelemetry.addData("Flipper", flipper.getServo() != null ? "✓" : "✗");
            multitelemetry.addData("Shooter", shooter.getMotor() != null ? "✓" : "✗");

            // Initialize Odometry
            if (ENABLE_ODOMETRY) {
                try {
                    odoLeft = hardwareMap.get(DcMotor.class, "ol");
                    odoRight = hardwareMap.get(DcMotor.class, "or");
                    odoPerp = hardwareMap.get(DcMotor.class, "perp");

                    odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    odoPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Thread.sleep(100);

                    odoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    odoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    odoPerp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    prevLeft = odoLeft.getCurrentPosition();
                    prevRight = odoRight.getCurrentPosition();
                    prevPerp = odoPerp.getCurrentPosition();

                    odometryInitialized = true;
                    multitelemetry.addData("Odometry", "✓");
                } catch (Exception e) {
                    odometryInitialized = false;
                    multitelemetry.addData("Odometry", "✗ - " + e.getMessage());
                }
            }
        } catch (Exception e) {
            multitelemetry.addData("ERROR", e.getMessage());
            multitelemetry.addData("Status", "Initialization Failed!");
        }

        multitelemetry.update();
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Get inputs
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // Toggle controls with debouncing
            if (gamepad1.a && !lastGamepad1A) drive.useFieldCentric = !drive.useFieldCentric;
            if (gamepad1.x && !lastGamepad1X) drive.useImuForFieldCentric = !drive.useImuForFieldCentric;
            if (gamepad1.right_bumper && !lastGamepad1RightBumper) drive.nerf = (drive.nerf == NORMAL_SPEED) ? SLOW_SPEED : NORMAL_SPEED;
            if (gamepad1.start && !lastGamepad1Start && drive.imuInitialized) drive.imu.resetYaw();
            if (gamepad1.back && !lastGamepad1Back && odometryInitialized) resetOdometry();

            lastGamepad1A = gamepad1.a;
            lastGamepad1X = gamepad1.x;
            lastGamepad1RightBumper = gamepad1.right_bumper;
            lastGamepad1Start = gamepad1.start;
            lastGamepad1Back = gamepad1.back;

            // Update odometry and drive
            if (odometryInitialized) updateOdometry();
            drive.update(true, forward, strafe, turn, odoHeading);

            // Intake controls
            if (gamepad2.left_bumper) {
                intakeLeft.goToPosition(-1.0);
                intakeRight.goToPosition(-1.0);
            } else if (gamepad2.right_bumper) {
                intakeLeft.goToPosition(1.0);
                intakeRight.goToPosition(1.0);
            } else {
                intakeLeft.stop();
                intakeRight.stop();
            }

            // Spindexer controls
            if (gamepad2.dpad_left) spindexer.goToPosition(-1);
            else if (gamepad2.dpad_right) spindexer.goToPosition(1);
            else spindexer.stop();

            // Flipper controls
            if (shooter.getTargetRPM() - shooter.getCurrentRPM() <= 50 && gamepad2.dpad_up) {
                flipper.goToPosition(0.5);
            } else if (gamepad2.dpad_down) {
                flipper.goToPosition(0);
            }

            // Shooter controls
            shooter.setTargetRPM(gamepad2.right_trigger >= 0.2 ? SHOOTER_ACTIVE_RPM : SHOOTER_IDLE_RPM);
            shooter.loop();

            // Telemetry
            multitelemetry.addData("Drive Mode", drive.useFieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
            if (drive.useFieldCentric) {
                multitelemetry.addData("Heading Source", drive.useImuForFieldCentric ? "IMU" : "ODOMETRY");
            }
            multitelemetry.addData("Speed", drive.nerf == SLOW_SPEED ? "SLOW" : "NORMAL");

            if (drive.useImuForFieldCentric && drive.imuInitialized) {
                multitelemetry.addData("IMU Heading", "%.2f°", Math.toDegrees(drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
            }

            if (odometryInitialized) {
                multitelemetry.addData("Odo Position", "X:%.2f Y:%.2f", xPos, yPos);
                multitelemetry.addData("Odo Heading", "%.2f°", Math.toDegrees(odoHeading));
            }

            // Component status
            String intakeStatus = gamepad2.left_bumper ? "IN" : (gamepad2.right_bumper ? "OUT" : "STOP");
            String spindexerStatus = gamepad2.dpad_left ? "LEFT" : (gamepad2.dpad_right ? "RIGHT" : "STOP");

            multitelemetry.addData("Intake", intakeStatus);
            multitelemetry.addData("Spindexer", spindexerStatus);
            multitelemetry.addData("Flipper Pos", "%.2f", flipper.getCurrentPosition());
            multitelemetry.addData("Shooter Target", "%.0f RPM", shooter.getTargetRPM());
            multitelemetry.addData("Shooter Current", "%.0f RPM %s", shooter.getCurrentRPM(), shooter.isAtTarget(50) ? "✓" : "✗");

            multitelemetry.addData("", "=== Controls ===");
            multitelemetry.addData("GP1 A", "Toggle Field/Robot");
            multitelemetry.addData("GP1 X", "Toggle IMU/Odo");
            multitelemetry.addData("GP1 RB", "Toggle Speed");
            multitelemetry.addData("GP1 Start/Back", "Reset IMU/Odo");
            multitelemetry.addData("GP2 Bumpers", "Intake");
            multitelemetry.addData("GP2 DPad L/R", "Spindexer");
            multitelemetry.addData("GP2 DPad U/D", "Flipper");
            multitelemetry.addData("GP2 RT", "Shooter");
            multitelemetry.update();

            sleep(20);
        }
    }

    private void updateOdometry() {
        int leftPos = -1 * odoLeft.getCurrentPosition();
        int rightPos = odoRight.getCurrentPosition();
        int perpPos = odoPerp.getCurrentPosition();

        int deltaLeft = leftPos - prevLeft;
        int deltaRight = rightPos - prevRight;
        int deltaPerp = perpPos - prevPerp;

        prevLeft = leftPos;
        prevRight = rightPos;
        prevPerp = perpPos;

        double dLeft = deltaLeft / TICKS_PER_INCH;
        double dRight = deltaRight / TICKS_PER_INCH;
        double dPerp = deltaPerp / TICKS_PER_INCH;

        double dHeading = (dRight - dLeft) / TRACK_WIDTH;
        double dForward = (dLeft + dRight) / 2.0;
        double dSide = dPerp - (dHeading * PERP_OFFSET);

        double sinH = Math.sin(odoHeading);
        double cosH = Math.cos(odoHeading);

        xPos += dForward * cosH - dSide * sinH;
        yPos += dForward * sinH + dSide * cosH;
        odoHeading += dHeading;
    }

    private void resetOdometry() {
        xPos = yPos = odoHeading = 0;

        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        try { Thread.sleep(100); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }

        odoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        prevLeft = odoLeft.getCurrentPosition();
        prevRight = odoRight.getCurrentPosition();
        prevPerp = odoPerp.getCurrentPosition();
    }
}