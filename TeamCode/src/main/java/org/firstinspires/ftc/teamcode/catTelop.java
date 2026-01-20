/* Copyright (c) 2025 FIRST. All rights reserved.
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
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@TeleOp(name = "Robot: Field Relative Mecanum Drive", group = "Robot")

public class catTelop extends LinearOpMode {

    // This declares the four motors needed
    CatHW_Async robot;
    CatLift lift;
    public  catTelop(){
        robot=new CatHW_Async();
    }

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;
    private Limelight3A limelight;
    boolean isAutoAim;

    double gatePos;

    double RPMs;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        lift = new CatLift(robot);
        lift.init();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);

        gatePos = 0.5;

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        isAutoAim=false;

        waitForStart();

        while (!isStopRequested()) {
            // If you press the left bumper, you get a drive from the point of view of the robot
            // (much like driving an RC vehicle)
            double driveSpeed;

            if (gamepad1.right_trigger > .1 || gamepad1.left_trigger > .1) {
                driveSpeed = 1.00;
            } else if (gamepad1.right_bumper || gamepad1.left_bumper) {
                driveSpeed = 0.3;
            } else {
                driveSpeed = 0.75;
            }
            robot.prowl.drive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, driveSpeed);

            if (gamepad1.square) {
                isAutoAim = true;
            }
            if (Math.abs(gamepad1.left_stick_x) > 0.1) {
                isAutoAim = false;
            }
            if (gamepad2.left_bumper) {
                robot.jaws.transfer(0.5);

            } else if (gamepad2.right_bumper) {
                robot.jaws.transfer(-0.5);
            } else {
                robot.jaws.transfer(0);
            }
            if (gamepad2.circle) {
                robot.jaws.intake.setPower(1);
            } else   if (gamepad2.square){
                robot.jaws.intake.setPower(-1);
            } else {
                robot.jaws.intake.setPower(0);
            }
            if (gamepad2.dpad_up) {
                robot.launch.setTargetRPM(robot.launch.targetRPM + 5);
            }
            if (gamepad2.dpad_down) {
                robot.launch.setTargetRPM(robot.launch.targetRPM - 5);
            }
            if (gamepad2.dpad_left) {
                robot.launch.setTargetRPM(3200);
            }
            if (gamepad2.dpad_right) {
                robot.launch.setTargetRPM(3700);
            }

            if (gamepad2.share) {
                lift.goUp();
            } else {
                lift.stopLift();
            }
            if (gamepad2.triangle){
                robot.jaws.gateClosed();
                //gatePos = gatePos + .003;
                //robot.jaws.gate.setPosition(gatePos);
            }
            if (gamepad2.cross){
                robot.jaws.gateOpen();
                //gatePos = gatePos - .003;
               // robot.jaws.gate.setPosition(gatePos);
            }


            LLStatus status = limelight.getStatus();
            robot.launch.setPowerFromPID();

            SparkFunOTOS.Pose2D currentPos = robot.prowl.myOtos.getPosition();

            telemetry.addData("Launch", " RPM: %5.0f target %.0f", robot.launch.getLastRPM(), robot.launch.targetRPM);

            telemetry.addData("gatePos","%.3f",gatePos);


            telemetry.addData("pos", "x: %4.1f y: %4.1f rot: %4.1f",currentPos.x,currentPos.y,currentPos.h);

            telemetry.addData("Name", "%s",
                        status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                        status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                        status.getPipelineIndex(), status.getPipelineType());
            telemetry.update();

            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                double xAngle = 0;

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",
                            fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    if ((fr.getFiducialId() == 20) || (fr.getFiducialId() == 24)) {
                        xAngle = fr.getTargetXDegrees();
                    }
                }
                if (isAutoAim) {
                    if (xAngle > 3) {
                        robot.prowl.drive(0, 0, 1, 0.4);
                    }
                    if (xAngle < -3) {
                        robot.prowl.drive(0, 0, -1, 0.4);
                    }

                }
            }
        }
    }





    // Thanks to FTC16072 for sharing this code!!

}
