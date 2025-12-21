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


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//ipomrt com.acmerobotics.roadrunner.geometry.Vector2d;

//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.roadrunner.drive.DriveSignal;
//import com.acmerobotics.roadrunner.drive.MecanumDrive;
//import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
//import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
//import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="edwardAutoRun", group="Old Auto")
@Disabled
public class edwardAutoTest1 extends LinearOpMode {
    private Servo leftClawOutake;
    private Servo intakeHinge;
    private Servo rightClawOutake;
    private Servo leftClawIntake;
    private Servo rightClawIntake;
    private DcMotor leftOutTakeArm;
    private DcMotor rightOutTakeArm;

    private Servo leftHingeOutake;
    private DcMotor Intakeextention = null;



    public void runOpMode() {
        leftClawOutake = hardwareMap.get(Servo.class, "leftClawOutake");
        rightClawOutake = hardwareMap.get(Servo.class, "rightClawOutake");
        leftClawIntake = hardwareMap.get(Servo.class, "leftClawIntake");
        rightClawIntake = hardwareMap.get(Servo.class, "rightClawIntake");
        leftOutTakeArm = hardwareMap.get(DcMotor.class, "leftOutTake");
        rightOutTakeArm = hardwareMap.get(DcMotor.class, "rightOutTake");
        intakeHinge = hardwareMap.get(Servo.class, "intakeHinge");
        Intakeextention = hardwareMap.get(DcMotor.class, "IntakeExtension");

//        rightHingeOutake = hardwareMap.get(Servo.class, "rightHingeOutake");
        leftHingeOutake = hardwareMap.get(Servo.class, "leftHingeOutake");

        leftOutTakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightOutTakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftOutTakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOutTakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftClawIntake.setDirection(Servo.Direction.REVERSE);
        intakeHinge.setDirection(Servo.Direction.REVERSE);
        leftOutTakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOutTakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftOutTakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightOutTakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftOutTakeArm.setTargetPosition(0);
        rightOutTakeArm.setTargetPosition(0);


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

        waitForStart();

//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
//        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
//
//        drive.setPoseEstimate(startPose);
//
//        Trajectory traj1 = drive.trajectoryBuilder(startPose)
//                .splineTo(new Vector2d(3, 34), Math.toRadians(-45))
//                .build();
//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                .forward(26.5)
//                .build();
//
//        Trajectory traj25 = drive.trajectoryBuilder(traj1.end())
//                .splineTo(new Vector2d(19.5, 22.8), Math.toRadians(0))
//                .build();
//
//        Trajectory traj3 = drive.trajectoryBuilder(traj25.end(),true)
//                .splineTo(new Vector2d(5,37.4), Math.toRadians(135))
//                .build();
//
//        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//                .splineTo(new Vector2d(19.5,34.5), Math.toRadians(0))
//                  .build();
//
//        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(),true)
//                .splineTo(new Vector2d(5,37.4), Math.toRadians(135))
//                .build();
//
//        Trajectory traj6 = drive.trajectoryBuilder(traj3.end(),true)
//                .splineTo(new Vector2d(50, -5), Math.toRadians(180))
//                .build();
//
//        Intakeextention.setPower(-0.4);
//        drive.followTrajectory(traj1);
//        outakeArm();
//        outakeDown();
//        drive.followTrajectory(traj25);
//        intakeToOutake();
//        drive.followTrajectory(traj3);
        outakeArm();
        outakeDown();
        //drive.followTrajectory(traj4);
        //intakeToOutake();
        //drive.followTrajectory(traj5);
        //outakeArm();
        //outakeDown();

    }
     private void intakeToOutake() {
        leftClawIntake.setPosition(0.5);
        rightClawIntake.setPosition(0.5);
        sleep(200);
        leftClawOutake.setPosition(0.55);
        rightClawOutake.setPosition(0.12);
        sleep(750);
        intakeHinge.setPosition(0.14);
        sleep(1450);
        leftClawIntake.setPosition(0);
        rightClawIntake.setPosition(0);
        sleep(750);
        leftHingeOutake.setPosition(0.5);
        sleep(500);
        intakeHinge.setPosition(0.96);
        sleep(500);
        leftHingeOutake.setPosition(0.92);
        sleep(500);
        leftClawOutake.setPosition(0.05);
        rightClawOutake.setPosition(0.35);
        sleep(1000);
        leftClawIntake.setPosition(0.5);
        rightClawIntake.setPosition(0.5);
        sleep(500);
    }

    private void outakeArm() {
        leftOutTakeArm.setTargetPosition(-3000);
        rightOutTakeArm.setTargetPosition(3000);
        leftOutTakeArm.setPower(0.9);
        rightOutTakeArm.setPower(0.9);
        sleep(1750);
        leftHingeOutake.setPosition(0);
        sleep(1450);
        leftClawOutake.setPosition(0.55);
        rightClawOutake.setPosition(0.12);
        sleep(750);
        leftClawOutake.setPosition(0.05);
        rightClawOutake.setPosition(0.35);
        sleep(700);
        leftHingeOutake.setPosition(0.92);
//        rightHingeOutake.setPosition(0.96);
        sleep(750);
        leftClawOutake.setPosition(0.55);
        rightClawOutake.setPosition(0.12);
    }

    private void outakeDown() {
        leftOutTakeArm.setTargetPosition(0);
        rightOutTakeArm.setTargetPosition(0);
        leftOutTakeArm.setPower(1);
        rightOutTakeArm.setPower(-1);
    }
}
