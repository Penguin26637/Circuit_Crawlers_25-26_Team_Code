//package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.MT;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.MT.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Rebuilt.MainPrograms.MotorPowerRegulator_New;
//
///**
// * INTO THE DEEP (2024-25) - Advanced Auto with Actions
// * Paste your trajectory AND add scoring actions!
// */
//@Config
//@Disabled
//@Autonomous(name = "INTO THE DEEP Advanced", group = "Competition")
//public class RoadRunnerAutoGregAdvanced extends LinearOpMode {
//
//    // Dashboard tunable
//    public static double START_X = -50.5;
//    public static double START_Y = -50.5;
//    public static double START_HEADING = 56.0;
//
//    // Hardware
//    private MecanumDrive drive;
//    private MultipleTelemetry multitelemetry;
//
//    // INTO THE DEEP subsystems
//    private IntakeSubsystem intake;
//    private ScoringSubsystem scorer;
//
//    @Override
//    public void runOpMode() {
//        multitelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        // Initialize
//        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING));
//        drive = new MecanumDrive(hardwareMap, startPose);
//
//        intake = new IntakeSubsystem(hardwareMap);
//        scorer = new ScoringSubsystem(hardwareMap, multitelemetry);
//
//        multitelemetry.addData("Status", "Ready!");
//        multitelemetry.update();
//
//        waitForStart();
//
//        if (opModeIsActive()) {
//            // Choose your autonomous strategy
//            runSampleBasketAuto();  // Change this to match your strategy
//        }
//    }
//
//    /**
//     * EXAMPLE 1: Just run the path (no actions)
//     */
//    private void runPathOnly() {
//        // PASTE YOUR TRAJECTORY HERE
//        Action trajectory = drive.actionBuilder(
//                        new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING))
//                )
//                .lineToLinearHeading(new Pose2d(-0.1, 0.1, Math.toRadians(225.9)))
//                .lineToLinearHeading(new Pose2d(-11.8, -35.8, Math.toRadians(-93.7)))
//                // ... rest of your path
//                .build();
//
//        Actions.runBlocking(trajectory);
//    }
//
//    /**
//     * EXAMPLE 2: Score in high basket (typical auto)
//     */
//    private void runSampleBasketAuto() {
//        Pose2d start = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING));
//
//        Action auto = new SequentialAction(
//                // Pre-load specimen - score in chamber
//                new ParallelAction(
//                        // Drive to chamber while raising arm
//                        drive.actionBuilder(start)
//                                .lineToLinearHeading(new Pose2d(-0.1, 0.1, Math.toRadians(225.9)))
//                                .build(),
//                        scorer.raiseToHighChamber()
//                ),
//                scorer.releaseSpecimen(),
//                scorer.lowerArm(),
//
//                // Get first sample
//                new ParallelAction(
//                        drive.actionBuilder(new Pose2d(-0.1, 0.1, Math.toRadians(225.9)))
//                                .lineToLinearHeading(new Pose2d(-11.8, -35.8, Math.toRadians(-93.7)))
//                                .build(),
//                        scorer.armToGround()
//                ),
//                intake.grab(),
//
//                // Score in high basket
//                new ParallelAction(
//                        drive.actionBuilder(new Pose2d(-11.8, -35.8, Math.toRadians(-93.7)))
//                                .lineToLinearHeading(new Pose2d(-11.9, -44.9, Math.toRadians(-97.3)))
//                                .build(),
//                        scorer.raiseToHighBasket()
//                ),
//                scorer.releaseSample(),
//
//                // Get second sample
//                new ParallelAction(
//                        drive.actionBuilder(new Pose2d(-11.9, -44.9, Math.toRadians(-97.3)))
//                                .lineToLinearHeading(new Pose2d(-12.1, -50.1, Math.toRadians(-97.3)))
//                                .build(),
//                        scorer.armToGround()
//                ),
//                intake.grab(),
//
//                // Score in high basket
//                new ParallelAction(
//                        drive.actionBuilder(new Pose2d(-12.1, -50.1, Math.toRadians(-97.3)))
//                                .lineToLinearHeading(new Pose2d(-12.4, -55.7, Math.toRadians(-97.3)))
//                                .build(),
//                        scorer.raiseToHighBasket()
//                ),
//                scorer.releaseSample(),
//
//                // Park
//                drive.actionBuilder(new Pose2d(-12.4, -55.7, Math.toRadians(-97.3)))
//                        .lineToLinearHeading(new Pose2d(36.0, -34.8, Math.toRadians(-93.2)))
//                        .build()
//        );
//
//        Actions.runBlocking(auto);
//    }
//
//    /**
//     * EXAMPLE 3: Your pasted path with actions added
//     * Copy your full path and add actions where needed
//     */
//    private void runCustomPath() {
//        Pose2d start = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING));
//
//        // PASTE YOUR FULL PATH HERE, then add actions between movements
//        Action auto = new SequentialAction(
//                // Movement 1
//                drive.actionBuilder(start)
//                        .lineToLinearHeading(new Pose2d(-0.1, 0.1, Math.toRadians(225.9)))
//                        .build(),
//                // ← Add action here if needed (e.g., scorer.scoreSpecimen())
//
//                // Movement 2
//                drive.actionBuilder(new Pose2d(-0.1, 0.1, Math.toRadians(225.9)))
//                        .lineToLinearHeading(new Pose2d(-11.8, -35.8, Math.toRadians(-93.7)))
//                        .build(),
//                // ← Add action here
//
//                // Movement 3
//                drive.actionBuilder(new Pose2d(-11.8, -35.8, Math.toRadians(-93.7)))
//                        .lineToLinearHeading(new Pose2d(-11.9, -44.9, Math.toRadians(-97.3)))
//                        .build()
//                // ← Add more movements...
//        );
//
//        Actions.runBlocking(auto);
//    }
//
//    // ========== SUBSYSTEMS ==========
//
//    /**
//     * Intake subsystem for grabbing balls
//     */
//    class IntakeSubsystem {
//        ServoClass intakeLeft, intakeRight, claw;
//
//        IntakeSubsystem(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
//            try {
//                intakeLeft = new ServoClass("intakeLeft", ServoClass.ServoType.CONTINUOUS_SERVO);
//                intakeRight = new ServoClass("intakeRight", ServoClass.ServoType.CONTINUOUS_SERVO);
//                claw = new ServoClass("claw", ServoClass.ServoType.STANDARD_SERVO);
//
//                intakeLeft.init(hardwareMap);
//                intakeRight.init(hardwareMap);
//                claw.init(hardwareMap);
//            } catch (Exception e) {
//                // Not all servos configured
//            }
//        }
//
//        Action grab() {
//            return packet -> {
//                if (intakeLeft != null) intakeLeft.goToPosition(-1.0);
//                if (intakeRight != null) intakeRight.goToPosition(-1.0);
//                sleep(500); // Run intake
//                if (claw != null) claw.goToPosition(0.7); // Close claw
//                sleep(300);
//                if (intakeLeft != null) intakeLeft.stop();
//                if (intakeRight != null) intakeRight.stop();
//                return false;
//            };
//        }
//
//        Action release() {
//            return packet -> {
//                if (claw != null) claw.goToPosition(0.0);
//                sleep(300);
//                return false;
//            };
//        }
//    }
//
//    /**
//     * Scoring subsystem for baskets and chambers
//     */
//    class ScoringSubsystem {
//        ServoClass wrist, elbow, claw;
//        MotorPowerRegulator_New arm;
//
//        ScoringSubsystem(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
//                         MultipleTelemetry telemetry) {
//            try {
//                wrist = new ServoClass("wrist", ServoClass.ServoType.STANDARD_SERVO);
//                elbow = new ServoClass("elbow", ServoClass.ServoType.STANDARD_SERVO);
//                claw = new ServoClass("claw", ServoClass.ServoType.STANDARD_SERVO);
//
//                wrist.init(hardwareMap);
//                elbow.init(hardwareMap);
//                claw.init(hardwareMap);
//
//                // Uncomment if you have a motorized arm
//                // arm = new MotorPowerRegulator_New(hardwareMap, telemetry, "arm");
//            } catch (Exception e) {
//                // Not configured
//            }
//        }
//
//        Action raiseToHighBasket() {
//            return packet -> {
//                if (wrist != null) wrist.goToPosition(0.9);
//                if (elbow != null) elbow.goToPosition(0.8);
//                sleep(800);
//                return false;
//            };
//        }
//
//        Action raiseToHighChamber() {
//            return packet -> {
//                if (wrist != null) wrist.goToPosition(0.7);
//                if (elbow != null) elbow.goToPosition(0.6);
//                sleep(800);
//                return false;
//            };
//        }
//
//        Action armToGround() {
//            return packet -> {
//                if (wrist != null) wrist.goToPosition(0.1);
//                if (elbow != null) elbow.goToPosition(0.0);
//                sleep(600);
//                return false;
//            };
//        }
//
//        Action lowerArm() {
//            return packet -> {
//                if (wrist != null) wrist.goToPosition(0.3);
//                if (elbow != null) elbow.goToPosition(0.2);
//                sleep(500);
//                return false;
//            };
//        }
//
//        Action releaseSample() {
//            return packet -> {
//                if (claw != null) claw.goToPosition(0.0);
//                sleep(300);
//                return false;
//            };
//        }
//
//        Action releaseSpecimen() {
//            return packet -> {
//                if (claw != null) claw.goToPosition(0.0);
//                sleep(200);
//                return false;
//            };
//        }
//    }
//}