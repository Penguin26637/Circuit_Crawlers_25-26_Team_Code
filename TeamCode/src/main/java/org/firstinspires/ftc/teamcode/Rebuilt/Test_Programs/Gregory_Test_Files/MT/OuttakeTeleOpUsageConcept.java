package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.MT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.vision.VisionPortal;


@TeleOp(name = "OuttakeTeleOpUsageConcept", group = "Test")
public class OuttakeTeleOpUsageConcept extends LinearOpMode {
//
//    private MultipleTelemetry multitelemetry;
//    private DigitalChannel limitSwitch;
//    private ServoClassMT Flipper, Spindexer;
//    private RedOuttakeDetectionSystem outtakeSystem;
//
//    // Flipper positions
//    private static final double FLIPPER_DOWN = 0.0;
//    private static final double FLIPPER_UP = 0.2;
//
//    // Spindexer speeds
//    private static final double SPINDEXER_FORWARD = 1.0;
//    private static final double SPINDEXER_STOP = 0.5;

    public CameraVisionOuttakeDetection cvos;
    public FtcDashboard dashboard;

    @Override
    public void runOpMode() {
//        multitelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        // Initialize hardware
//        try {
//            limitSwitch = hardwareMap.get(DigitalChannel.class, "touch");
//            limitSwitch.setMode(DigitalChannel.Mode.INPUT);
//        } catch (Exception e) {
//            multitelemetry.addData("Error", "Limit switch not found!");
//        }
//
//        Flipper = safeInitServo("Flipper", ServoClassMT.ServoType.STANDARD_SERVO);
//        Spindexer = safeInitServo("Spindexer", ServoClassMT.ServoType.CONTINUOUS_SERVO);
//
//        // Initialize red detection system
//        outtakeSystem = new RedOuttakeDetectionSystem(hardwareMap);
//
////        if (!outtakeSystem.isInitialized()) {
////            multitelemetry.addData("Error", "Outtake system failed to initialize!");
////        } else {
////            multitelemetry.addData("Camera", "Streaming to Dashboard");
////            // Ensure camera stream is started
////            outtakeSystem.startCameraStream();
////
////            // OPTIONAL: Configure ROI to focus on bottom half of frame
////            // Uncomment and adjust these values as needed:
////            // RedOuttakeDetectionSystem.roiLeftPercent = 0.0;
////            // RedOuttakeDetectionSystem.roiTopPercent = 0.5;
////            // RedOuttakeDetectionSystem.roiRightPercent = 1.0;
////            // RedOuttakeDetectionSystem.roiBottomPercent = 1.0;
////        }
//
//        multitelemetry.addData("Status", "Ready! Press Play");
//        multitelemetry.addLine();
//        multitelemetry.addData("Controls", "Automatic red detection controls flipper");
//        multitelemetry.addData("Limit Switch", "Stops spindexer when pressed");
//        multitelemetry.addData("ROI Config", "Adjust via FTC Dashboard");
//        multitelemetry.addData("Dashboard URL", "http://192.168.43.1:8080");
//        multitelemetry.update();

//        outtakeSystem.startCameraStream();
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();


        cvos = new CameraVisionOuttakeDetection(hardwareMap, dashboard);
        while (cvos.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_READY) {
            packet.put("Camera State", "Not Ready");
            dashboard.sendTelemetryPacket(packet);
        }
        packet.put("Camera State", "Ready");
        dashboard.sendTelemetryPacket(packet);

        cvos.startCameraStreamSystem();

        waitForStart();

        // Start camera stream again after play is pressed
//        if (outtakeSystem.isInitialized()) {
//            outtakeSystem.startCameraStream();
//        }
        packet.clearLines();
        dashboard.sendTelemetryPacket(packet);
        cvos.configureCameraControls();

        while (opModeIsActive()) {
//            packet.put("Purple Blobs: ", cvos.getPurpleBlobs());
//            packet.put("Green Blobs: ", cvos.getGreenBlobs());
            if(!cvos.getPurpleBlobs().isEmpty()) {
                packet.put("Largest Purple Blob", cvos.getPurpleBlobs().get(0).getContourArea());
            }
            if(!cvos.getGreenBlobs().isEmpty()) {
                packet.put("Largest Green Blob", cvos.getGreenBlobs().get(0).getContourArea());
                packet.put("Color Detected Green: ", cvos.detectClosestObjectAndDecide("Green"));
            }
            dashboard.sendTelemetryPacket(packet);

            // Update the red detection system
//            outtakeSystem.update();
//
//            // Check limit switch state
//            boolean limitSwitchPressed = isLimitSwitchPressed();
//
//            // FLIPPER CONTROL - Based on red detection
//            if (outtakeSystem.shouldFlipUp()) {
//                // Red detected - flip up
//                safeGoToPosition(Flipper, FLIPPER_UP);
//            } else {
//                // No red - keep flipper down
//                safeGoToPosition(Flipper, FLIPPER_DOWN);
//            }
//
//            // SPINDEXER CONTROL - Based on limit switch
//            if (limitSwitchPressed) {
//                // Limit switch pressed - stop spindexer
//                safeStop(Spindexer);
//            } else {
//                // Limit switch not pressed - run spindexer forward
//                safeGoToPosition(Spindexer, SPINDEXER_FORWARD);
//            }
//            multitelemetry.addData("Red Blobs", outtakeSystem.getRedBlobs() + "");
//            multitelemetry.update();
//            // Update telemetry
//            updateTelemetry(limitSwitchPressed);
//
//            sleep(50);  // Small delay to prevent loop overload
//        }
//
//        // Cleanup
//        outtakeSystem.stopCameraStream();
//        outtakeSystem.stop();
//       }
        }

//    private ServoClassMT safeInitServo(String name, ServoClassMT.ServoType type) {
//        try {
//            ServoClassMT servo = new ServoClassMT(name, type);
//            servo.init(hardwareMap);
//            return servo;
//        } catch (Exception e) {
//            multitelemetry.addData("Error", name + " not found!");
//            multitelemetry.update();
//            return null;
//        }
//    }
//
//    private void safeGoToPosition(ServoClassMT servo, double position) {
//        if (servo != null) servo.goToPosition(position);
//    }
//
//    private void safeStop(ServoClassMT servo) {
//        if (servo != null) servo.stop();
//    }
//
//    private boolean isLimitSwitchPressed() {
//        if (limitSwitch == null) return false;
//        return limitSwitch.getState();
//    }
//
//    private void updateTelemetry(boolean limitPressed) {
//        // Call the outtake system's telemetry (this updates camera stream)
//        outtakeSystem.sendTelemetry(multitelemetry);
//
//        multitelemetry.addLine();
//        multitelemetry.addLine("=== SYSTEM STATUS ===");
//        multitelemetry.addData("Limit Switch", limitPressed ? "PRESSED" : "Not Pressed");
//        multitelemetry.addLine();
//
//        // Action telemetry
//        multitelemetry.addLine("=== ACTIONS ===");
//        multitelemetry.addData("Flipper", outtakeSystem.shouldFlipUp() ? "UP" : "DOWN");
//        multitelemetry.addData("Spindexer", limitPressed ? "STOPPED" : "RUNNING");
//        multitelemetry.addLine();
//
//        // Dashboard info
//        multitelemetry.addData("Dashboard", "Camera feed @ 192.168.43.1:8080");
//
//        multitelemetry.update();
//    }
    }
}