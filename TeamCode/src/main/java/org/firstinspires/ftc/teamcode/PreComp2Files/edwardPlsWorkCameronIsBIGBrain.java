/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.PreComp2Files;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModkes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="edwardPlsWorkCameronIsBIGBrain", group="Old Auto")
@Disabled
public class edwardPlsWorkCameronIsBIGBrain extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor Intakeextention = null;


//    private CRServo Intakerotate;
//    private Servo intakeClaw;
    private Servo leftClawOutake;
    private Servo intakeHinge;
    private Servo rightClawOutake;
    private Servo leftClawIntake;
    private Servo rightClawIntake;
    private DcMotor leftOutTakeArm;
    private DcMotor rightOutTakeArm;

    private Servo leftHingeOutake;
    private Servo rightHingeOutake;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //Drive
        backLeftDrive  = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");


        //Arm Controls - IntakeArms
        Intakeextention = hardwareMap.get(DcMotor.class, "IntakeExtension");

        //outtakeControls
        leftClawOutake = hardwareMap.get(Servo.class, "leftClawOutake");
        rightClawOutake = hardwareMap.get(Servo.class, "rightClawOutake");
        leftClawIntake = hardwareMap.get(Servo.class, "leftClawIntake");
        rightClawIntake = hardwareMap.get(Servo.class, "rightClawIntake");
        leftOutTakeArm = hardwareMap.get(DcMotor.class, "leftOutTake");
        rightOutTakeArm = hardwareMap.get(DcMotor.class, "rightOutTake");
        intakeHinge = hardwareMap.get(Servo.class, "intakeHinge");

//        rightHingeOutake = hardwareMap.get(Servo.class, "rightHingeOutake");
        leftHingeOutake = hardwareMap.get(Servo.class, "leftHingeOutake");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives ma1111111111y require direction flips
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);


//        rightClawIntake.setDirection(Servo.Direction.REVERSE);
//        rightHingeOutake.setDirection(Servo.Direction.REVERSE);
        //leftClawOutake.setDirection(Servo.Direction.REVERSE);
        leftClawIntake.setDirection(Servo.Direction.REVERSE);
        intakeHinge.setDirection(Servo.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftOutTakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightOutTakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftOutTakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOutTakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOutTakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOutTakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftOutTakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightOutTakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftOutTakeArm.setTargetPosition(0);
//        rightOutTakeArm.setTargetPosition(0);


//        leftOutTakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightOutTakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftOutTakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightOutTakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftOutTakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightOutTakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (leftOutTakeArm.getCurrentPosition() != 0 && rightOutTakeArm.getCurrentPosition() != 0) {
            idle();
        }
        leftClawIntake.setPosition(0);
        rightClawIntake.setPosition(0);

        sleep(500);

        intakeHinge.setPosition(0.96);

        sleep(500);

        leftHingeOutake.setPosition(0.92);
//        rightHingeOutake.setPosition(0.96);
        sleep(750);

        leftClawOutake.setPosition(0.05);
        rightClawOutake.setPosition(0.35);
        sleep(1000);
        leftClawIntake.setPosition(0.5);
        rightClawIntake.setPosition(0.5);
//        leftClawOutake.setPosition(0.5);
//        rightClawOutake.setPosition(0);

        boolean is_pressed_Hinge = true;
        boolean is_pressed_intake = false;
        boolean is_outake_down = true;
        boolean is_outake_open = false;
        boolean if_speciman_grab = false;
        boolean hikingMode= false;
        double nerf = 0.75;

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            //Movement
            //Claw
            double ExtentionPower = -0.85 * gamepad2.left_stick_y;

            //TEST MOVE IN RESPECT TO DRIVER

            //Get controller values
            double cAngle = Math.atan(-gamepad1.left_stick_y/-gamepad1.left_stick_x)+Math.PI/2;
            double speed = Math.sqrt(Math.abs(gamepad1.left_stick_y+gamepad1.left_stick_x));

            //Get robot angle
            double trackWidth = 14.5;
            double rightOD = (backRightDrive.getCurrentPosition()/2000.0)*Math.PI*1.75;
            double leftOD = (backLeftDrive.getCurrentPosition()/2000.0)*Math.PI*1.75;
            double difference = leftOD - rightOD;
            double heading = ((Math.PI*2)*difference)/(2.0*Math.PI*trackWidth);

            //Get target angle
            double tAngle = cAngle - heading + (3.0/2.0) * Math.PI;

            //Return values to usable numbers
            double Logdrive = Math.sin(tAngle)*speed*nerf;
            double LATdrive = Math.cos(tAngle)*speed*nerf;
            double Turndrive = -gamepad1.right_stick_x*0.60;

            //outakeArms
            //double outakeArmPower = 0.8 * gamepad2.right_stick_y;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
            // Drive
            backLeftDrive.setPower(Logdrive-LATdrive-Turndrive);
            backRightDrive.setPower(Logdrive+LATdrive+Turndrive);
            frontLeftDrive.setPower(Logdrive+LATdrive-Turndrive);
            frontRightDrive.setPower(Logdrive-LATdrive+Turndrive);
            Intakeextention.setPower(ExtentionPower);

            //leftOutTakeArm.setPower(outakeArmPower);
            //rightOutTakeArm.setPower(-outakeArmPower);
            // the claw values are for testing and have not been finalized  12/20/24



//            if (gamepad1.right_stick_y >= 0.01) {
//                arm.setTargetPosition(upPosition);
//            } else if (gamepad2.y) {
//                arm.setTargetPosition(downPosition);
//            }
//            if (arm.isBusy()) {
//                arm.setPower(1);
//            } else {
//                arm.setPower(0);
//            }

            leftOutTakeArm.setPower(0.8*gamepad2.right_stick_y);
            rightOutTakeArm.setPower(0.8* -gamepad2.right_stick_y);

            if(gamepad1.x && !hikingMode) {
                hikingMode = true;
                sleep(250);
            }

            if(hikingMode) {
                leftOutTakeArm.setPower(1);
                rightOutTakeArm.setPower(-1);
            }


            if(gamepad1.left_bumper) {
                nerf = 0.4;
            }
            if(gamepad1.right_bumper) {
                nerf = 0.75;
            }

            if(gamepad2.x && is_outake_open) {
//                nerf = 0;
                leftClawOutake.setPosition(0.05);
                rightClawOutake.setPosition(0.35);
                sleep(1000);
                leftClawIntake.setPosition(0.5);
                rightClawIntake.setPosition(0.5);
                sleep(250);
//                leftHingeOutake.setPosition(0.4);
////                rightHingeOutake.setPosition(0.4);
                is_outake_open = false;
            }

//            nerf = 0.75;

            if(gamepad2.x && !is_outake_open) {
//                nerf = 0;
                leftClawOutake.setPosition(0.5);
                rightClawOutake.setPosition(0.12);
                sleep(  250);
//                leftClawIntake.setPosition(0.3);
//                rightClawIntake.setPosition(0.3);
//                intakeHinge.setPosition(0.5);
//                sleep(750);
//                leftHingeOutake.setPosition(0.4);
////                rightHingeOutake.setPosition(0.4);
                is_outake_open = true;
            }
//            nerf = 0.75;

            if(is_outake_down && !is_outake_open && gamepad2.y) {
//                nerf = 0;
                leftHingeOutake.setPosition(0);
//                rightHingeOutake.setPosition(0.2);
                sleep(250);
//                leftClawOutake.setPosition(0.8);
//                rightClawOutake.setPosition(0.72);
//                sleep(800);
//                leftHingeOutake.setPosition(0.94);
////                rightHingeOutake.setPosition(0.94);
//                sleep(200);
                is_outake_down = false;
            }

//            nerf = 0.75;

            if(!is_outake_down && !is_outake_open && gamepad2.y) {
//                nerf = 0;
                leftHingeOutake.setPosition(0.92);
//                rightHingeOutake.setPosition(0.2);
                sleep(250);
//                leftClawOutake.setPosition(0.8);
//                rightClawOutake.setPosition(0.72);
//                sleep(800);
//                leftHingeOutake.setPosition(0.94);
////                rightHingeOutake.setPosition(0.94);
//                sleep(200);
                is_outake_down = true;
            }

//

            //operateClaw
            if(is_pressed_intake && gamepad2.b) {
//                nerf = 0;
                leftClawIntake.setPosition(0.5);
                rightClawIntake.setPosition(0.5);
                is_pressed_intake = false;
                sleep(250);
//                nerf = 0.75;
            } else if(!is_pressed_intake && gamepad2.b) {
/*                nerf = 0;*/
                leftClawIntake.setPosition(0);
                rightClawIntake.setPosition(0);
                is_pressed_intake = true;
                sleep(250);
//                nerf = 0.75;
            }
//            nerf = 0.75;
            //operateClawHinge and Outake - Intake Handoff
            if(!is_pressed_Hinge &&  is_outake_open && gamepad2.a) {
//                nerf = 0;
                leftHingeOutake.setPosition(0.5);
                sleep(500);
                intakeHinge.setPosition(0.96);
                sleep(500);
                leftHingeOutake.setPosition(0.92);
                is_pressed_Hinge = true;
                sleep(300);
//                nerf = 0.75;
            } else if(is_pressed_Hinge && gamepad2.a) {
//                nerf = 0;
                intakeHinge.setPosition(0.14);
                is_pressed_Hinge = false;
                sleep(250);
//                leftClawOutake.setPosition(0.85);
//                rightClawOutake.setPosition(0.85);
//                sleep(200);
//                leftClawIntake.setPosition(0.3);
//                rightClawIntake.setPosition(0.3);
//                intakeHinge.setPosition(0.5);
//                sleep(750);
//                leftHingeOutake.setPosition(0.4);
//                rightHingeOutake.setPosition(0.4);
//                is_outake_down = false;
//                nerf = 0.75;
            }

//            nerf = 0.75 ;

            if(gamepad2.right_bumper && is_outake_down && is_outake_open && !is_pressed_intake && !is_pressed_Hinge) {
//                nerf =0;
                leftClawIntake.setPosition(0);
                rightClawIntake.setPosition(0);
                sleep(750);
                leftHingeOutake.setPosition(0.5);
                sleep(500);
                intakeHinge.setPosition(0.96);
                sleep(500);
                leftHingeOutake.setPosition(0.92);
                sleep(750);
                leftClawOutake.setPosition(0.05);
                rightClawOutake.setPosition(0.35);
                sleep(1000);
                leftClawIntake.setPosition(0.5);
                rightClawIntake.setPosition(0.5);
                sleep(750);
                leftHingeOutake.setPosition(0.4);

                is_pressed_intake = false;
                is_pressed_Hinge= true;
                is_outake_down = true;
                is_outake_open = false;




            }
//            if(!if_speciman_grab && gamepad2.right_bumper) {
//                nerf = 0;
//                if_speciman_grab = true;
//                is_outake_down = false;
//                leftClawOutake.setPosition(0.8);
//                rightClawOutake.setPosition(0.72);
//                sleep(200);
//                leftHingeOutake.setPosition(0.1);
//                rightHingeOutake.setPosition(0.1);
//                sleep(2000);
////                leftClawOutake.setPosition(0.94);
////                rightClawOutake.setPosition(0.94);
////                sleep(500);
//                nerf = 0.75;
//
//            } else if (if_speciman_grab && gamepad2.right_bumper) {
//                nerf = 0;
//                if_speciman_grab = false;
//                leftClawOutake.setPosition(0.94);
//                rightClawOutake.setPosition(0.94);
//                sleep(500);
//                nerf = 0.75;
//            }

//            nerf = 0.75;


                // Show the elapsed game time and wheel power.
            telemetry.addData("Extension - Power", ExtentionPower);
            telemetry.addData("Current Position Hinge-Intake", intakeHinge.getPosition());
            telemetry.addData("Current Position Intake Left Claw", leftClawIntake.getPosition());
            telemetry.addData("Current Position Intake Right Claw", rightClawIntake.getPosition());
            telemetry.addData("Nerf Constant: ", nerf);
            telemetry.addData("Intake Pressed", is_pressed_intake);
            telemetry.addData("JoyStickValue", gamepad2.right_stick_y);
            telemetry.addData("Heading", heading);
            telemetry.addData("Difference", difference);
            telemetry.addData("Y Val", Logdrive);
            telemetry.addData("X Val", LATdrive);
            telemetry.addData("Controller Angle", cAngle);
            telemetry.addData("Target Value", tAngle);
            telemetry.addData("Left OD", backLeftDrive.getCurrentPosition());
//            elemetry.addData("State of Button: ", is_pressed_Hinge);

            telemetry.update();



        }
    }
}



