package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.MT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Rebuilt.MainPrograms.MotorPowerRegulator_New;
import org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.MT.MecanumDrive;
import org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.MT.ServoClassMT;

@Autonomous(name = "INTO THE DEEP Auto", group = "Competition")
@Disabled
public class RoadRunnerAutoGreg extends LinearOpMode {

    // Hardware
    private MecanumDrive drive;
    private MultipleTelemetry multitelemetry;
    private DigitalChannel limitSwitch;

    // Subsystems
    private ServoClassMT intakeLeft, intakeRight;
    private ServoClassMT Spindexer, Flipper;
    private MotorPowerRegulator_New arm;

    private static int ball;

    // STARTING POSITION
    private static final Pose2d START_POSE = new Pose2d(-50.5, -50.5, Math.toRadians(56.0));

    // Ball positions
    private static final double SPINDEXER_POS_BALL1 = 0.7;
    private static final double SPINDEXER_POS_BALL2 = 0.25;
    private static final double SPINDEXER_POS_BALL3 = 0.1;

    @Override
    public void runOpMode() {
        multitelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeHardware();
        drive = new MecanumDrive(hardwareMap, START_POSE);

        multitelemetry.addData("Status", "Ready!");
        multitelemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            runAutonomousSequence();
        }
    }

    private void runAutonomousSequence() {
        Action trajectory = drive.actionBuilder(START_POSE)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-0.1, 0.1, Math.toRadians(225.9)), 0)
                .splineToLinearHeading(new Pose2d(-11.8, -35.8, Math.toRadians(-93.7)), 0)
                .splineToLinearHeading(new Pose2d(-11.9, -44.9, Math.toRadians(-97.3)), 0)
                .splineToLinearHeading(new Pose2d(-12.1, -50.1, Math.toRadians(-97.3)), 0)
                .splineToLinearHeading(new Pose2d(-12.4, -55.7, Math.toRadians(-97.3)), 0)
                .splineToLinearHeading(new Pose2d(-0.1, 0.1, Math.toRadians(225.9)), 0)
                .splineToLinearHeading(new Pose2d(11.9, -33.4, Math.toRadians(-89.1)), 0)
                .splineToLinearHeading(new Pose2d(11.5, -42.4, Math.toRadians(-89.1)), 0)
                .splineToLinearHeading(new Pose2d(11.6, -47.7, Math.toRadians(-89.1)), 0)
                .splineToLinearHeading(new Pose2d(11.3, -54.4, Math.toRadians(-89.1)), 0)
                .splineToLinearHeading(new Pose2d(-0.1, 0.1, Math.toRadians(225.9)), 0)
                .splineToLinearHeading(new Pose2d(36.0, -34.8, Math.toRadians(-93.2)), 0)
                .build();

        Actions.runBlocking(trajectory);

        multitelemetry.addData("Status", "Path Complete!");
        multitelemetry.update();
    }

    private void initializeHardware() {
        intakeLeft = safeInitServo("intakeLeft", ServoClassMT.ServoType.CONTINUOUS_SERVO);
        intakeRight = safeInitServo("intakeRight", ServoClassMT.ServoType.CONTINUOUS_SERVO);

        Spindexer = safeInitServo("Spindexer", ServoClassMT.ServoType.CONTINUOUS_SERVO);
        Flipper = safeInitServo("Flipper", ServoClassMT.ServoType.STANDARD_SERVO);

        try {
            limitSwitch = hardwareMap.get(DigitalChannel.class, "touch");
            limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception ignored) {}
    }

    private ServoClassMT safeInitServo(String name, ServoClassMT.ServoType type) {
        try {
            ServoClassMT servo = new ServoClassMT(name, type);
            servo.init(hardwareMap);
            return servo;
        } catch (Exception e) {
            return null;
        }
    }

    private void safeGoToPosition(ServoClassMT servo, double position) {
        if (servo != null) servo.goToPosition(position);
    }

    private Action grabBall() {
        return packet -> {
            if (Spindexer != null) {
                switch (ball) {
                    case 1 -> safeGoToPosition(Spindexer, SPINDEXER_POS_BALL1);
                    case 2 -> safeGoToPosition(Spindexer, SPINDEXER_POS_BALL2);
                    case 3 -> safeGoToPosition(Spindexer, SPINDEXER_POS_BALL3);
                }
            }
            safeGoToPosition(intakeLeft, 1);
            safeGoToPosition(intakeRight, 1);
            sleep(300);
            return false;
        };
    }

    private Action releaseBall() {
        return packet -> {
            shoot();
            sleep(300);
            return false;
        };
    }

    private boolean isLimitSwitchPressed() {
        if (limitSwitch == null) return false;
        boolean rawState = limitSwitch.getState();
        boolean pressed = rawState;

        updateTelemetry("Limit Switch", rawState, pressed);

        sleep(50);
        return pressed;
    }

    private void updateTelemetry(String name, boolean raw, boolean pressed) {
        multitelemetry.addData(name + " Raw", raw);
        multitelemetry.addData(name + " Pressed", pressed);
        multitelemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put(name + " Raw", raw);
        packet.put(name + " Pressed", pressed);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    private boolean shoot() {
        if (isLimitSwitchPressed()) {
            Spindexer.stop();
            safeGoToPosition(Flipper, 0.2);
            sleep(5);
            safeGoToPosition(Flipper, 0);
            return true;
        } else {
            safeGoToPosition(Spindexer, 1);
            return false;
        }
    }
}
