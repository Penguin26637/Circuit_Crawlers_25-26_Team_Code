package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.ObjectDetectionExamplesTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Example TeleOp that uses the ObeliskIntakeSystem class
 * 
 * This demonstrates how to integrate the intake system into your existing TeleOp code
 */
@TeleOp(name = "TeleOp with Intake Systemasdf", group = "Examples")
public class ExampleTeleOp extends LinearOpMode {
    
    // Your existing robot hardware
//    private DcMotor leftDrive;
//    private DcMotor rightDrive;
//    private DcMotor intakeMotor;
    
    // The intake decision system - just one line!
    private ObeliskIntakeSystem intakeSystem;
    
    @Override
    public void runOpMode() {
        
        // Initialize your existing hardware
//        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
//        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
//        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        
        // Initialize the intake system - just one line!
        intakeSystem = new ObeliskIntakeSystem(hardwareMap);
        
        // Check if it initialized properly
        if (!intakeSystem.isInitialized()) {
            telemetry.addData("ERROR", "Intake system failed to initialize!");
            telemetry.update();
        }
        intakeSystem.resetBallCounter();
        
        telemetry.addData("Status", "Ready to start!");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            // ========== YOUR EXISTING DRIVE CODE ==========
//            double drive = -gamepad1.left_stick_y;
//            double turn = gamepad1.right_stick_x;
//
//            leftDrive.setPower(drive + turn);
//            rightDrive.setPower(drive - turn);
            
            // ========== INTAKE SYSTEM - JUST TWO LINES! ==========
            
            // Update the system (detects obelisk and balls)
            intakeSystem.update();

            
            // Control intake based on the decision
            if (intakeSystem.shouldPickup()) {
                telemetry.addLine("intake");
            } else {
                telemetry.addLine("intake");
            }
            
//            // Optional: Manual override with gamepad button
//            if (gamepad1.a) {
//                intakeMotor.setPower(1.0);  // Force intake on
//            } else if (gamepad1.b) {
//                intakeMotor.setPower(-1.0); // Force reverse
//            }
            
            // ========== TELEMETRY ==========
            
            // Your existing telemetry
//            telemetry.addData("Drive", "%.2f", drive);
//            telemetry.addData("Turn", "%.2f", turn);
            
            // Intake system telemetry (just one line!)
            intakeSystem.sendTelemetry(telemetry);
            
            telemetry.update();
            sleep(20);
        }
        
        // Cleanup
        intakeSystem.stop();
    }
}
