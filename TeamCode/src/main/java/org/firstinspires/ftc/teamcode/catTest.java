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

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
@TeleOp(name = "Launch Test", group = "Robot")
@Config
public class catTest extends OpMode {
    // This declares the four motors needed

    public DcMotorEx launcher;
    private static final int ticksPerRev=28;
    private ElapsedTime timer = new ElapsedTime();
    private int lastPos = 0;
    private double lastTime = 0;

    public static double kP = 0.0008;
    public static double kI = 0.0;
    public static double kD = 0.0001;
    public static double kF = 0.0;
    public static double targetRPM;
    private double integral = 0;
    private double lastError = 0;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;
    private Limelight3A limelight;
    boolean isAutoAim;

    public void reset(){ integral = 0; lastError = 0; timer.reset();}
    public double update (double measuredRPM){
        double dt = Math.max(1e-3,timer.seconds());
        timer.reset();
        double error = targetRPM - measuredRPM;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;
        double out = kP * error + kI * integral + kD * derivative + kF * targetRPM;
        return Range.clip(out,-1.0,1.0);

    }

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    @Override
    public void init() {
        launcher = (DcMotorEx) hardwareMap.dcMotor.get("launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastPos = launcher.getCurrentPosition();
        lastTime = timer.seconds();
        targetRPM = 0;
    }

    @Override
    public void loop() {
        //launcher.setPower(gamepad1.right_stick_y);
        if (gamepad2.dpad_up){
            targetRPM += 1;
        }
        if (gamepad2.dpad_down){
            targetRPM -= 1;
        }
        if (targetRPM < 0){
            targetRPM = 0;
        }
        if (targetRPM > 6000){
            targetRPM = 6000;
        }
        launcher.setVelocity(targetRPM * ticksPerRev / 60);
        double RPM = getRPM();
        double vel = launcher.getVelocity();
        telemetry.addData("Launch", "power: %.2f RPM: %5.0f %5.0f target %.0f",gamepad1.right_stick_y,RPM,vel/ticksPerRev,targetRPM);

        TelemetryPacket packet = new TelemetryPacket();
        dashboardTelemetry.addData("Launch", "power: %.2f RPM: %5.0f %5.0f target %.0f",gamepad1.right_stick_y,RPM,vel/ticksPerRev,targetRPM);
        dashboardTelemetry.addData("RPM",RPM);
        dashboardTelemetry.update();



    }
    private double getRPM(){
        int currentPos = launcher.getCurrentPosition();
        double currentTime = timer.seconds();
        int deltaTicks = currentPos - lastPos;
        double deltaTime = currentTime - lastTime;
        if (deltaTime <= 0) return 0;
        double rev = (double) deltaTicks/ticksPerRev;
        double RPM = (rev/deltaTime) * 60;
        telemetry.addData("Launch", "dT = %4.3f dTick %4d\n",deltaTime,deltaTicks);
        lastPos = currentPos;
        lastTime = currentTime;
        return RPM;
    }






}
