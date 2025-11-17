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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

public class catTelop extends OpMode {
    // This declares the four motors needed
    CatHW_Async robot;
    public  catTelop(){
        robot=new CatHW_Async();
    }


    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;
    private Limelight3A limelight;
    boolean isAutoAim;

    @Override
    public void init() {

        robot.init(hardwareMap, this);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        isAutoAim=false;
    }

    @Override
    public void loop() {

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

        if (gamepad1.square){
            isAutoAim=true;
        }
        if (Math.abs(gamepad1.left_stick_x)>0.1) {
            isAutoAim=false;
        }
        if (gamepad2.circle){
            robot.jaws.intake.setPower(1);
        } else  {
            robot.jaws.intake.setPower(0);
        }
        if (gamepad2.dpad_up){
            robot.jaws.targetRPM += 1;
        }
        if (gamepad2.dpad_down){
            robot.jaws.targetRPM -= 1;
        }
        if (robot.jaws.targetRPM < 0){
            robot.jaws.targetRPM = 0;
        }
        if (robot.jaws.targetRPM > 6000){
            robot.jaws.targetRPM = 6000;
        }
        robot.jaws.launcher.setVelocity(robot.jaws.targetRPM * robot.jaws.ticksPerRev / 60);
        double RPM = robot.jaws.getRPM();
        double vel = robot.jaws.launcher.getVelocity();
        telemetry.addData("Launch", "power: %.2f RPM: %5.0f %5.0f target %.0f",gamepad1.right_stick_y,RPM,vel/robot.jaws.ticksPerRev,robot.jaws.targetRPM);

        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            double xAngle=0;

            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",
                        fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                if ((fr.getFiducialId()==20)||(fr.getFiducialId()==24)){
                    xAngle=fr.getTargetXDegrees();
                }
            }
            if (isAutoAim){
                if (xAngle>5){
                   robot.prowl.drive(0,0,1,0.3);
                }
                if (xAngle<-5){
                    robot.prowl.drive(0,0,-1,0.3);
                }

            }
        }

        robot.jaws.launch(gamepad1.right_trigger);

        //telemetry.addData("Launch", "power: %.2f rpm: %.1f",gamepad1.right_trigger, robot.jaws.launchRPM());

    }





    // Thanks to FTC16072 for sharing this code!!

}
