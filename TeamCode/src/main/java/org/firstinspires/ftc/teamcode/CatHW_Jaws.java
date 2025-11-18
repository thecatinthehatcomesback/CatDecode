package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * CatHW_Jaws.java
 *
 *
 * This class containing common code accessing hardware specific to the movement of the jaws/intake.
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_Jaws extends CatHW_Subsystem
{

    // Motors: //

    public DcMotor intake = null;
    public DcMotorEx launcher;

    public DcMotor transfer;
    public ElapsedTime liftTime = null;
    public ElapsedTime pidTimer = null;
    //values for pid
    public static final int ticksPerRev=28;
    private int lastPos = 0;
    public static double targetRPM;
    double lastTime;
    // Timers: //
    private ElapsedTime timer = new ElapsedTime();


    /* Constructor */
    public CatHW_Jaws(CatHW_Async mainHardware) {
        super(mainHardware);

    }


    /* Initialize standard Hardware interfaces */
    public void init() {
        // Define and initialize motors: /armMotor/


        launcher=(DcMotorEx) hwMap.dcMotor.get("launcher");
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake=hwMap.dcMotor.get("intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastPos = launcher.getCurrentPosition();
        lastTime = timer.seconds();
        targetRPM = 0;

        transfer = hwMap.dcMotor.get("transfer");
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftTime = new ElapsedTime();
        pidTimer = new ElapsedTime();
    }

    public void transfer(double speed){
        transfer.setPower(speed);
    }
    public void bumpRPM(){
        targetRPM += 1;
        if (targetRPM > 6000){
            targetRPM = 6000;
        }
        setVelocity();
    }
    public void decRPM(){
        targetRPM -= 1;
        if (targetRPM < 0){
            targetRPM = 0;
        }
        setVelocity();
    }
    public void setVelocity() {
        launcher.setVelocity(targetRPM * ticksPerRev / 60);
    }
    public double getRPM(){
        int currentPos = launcher.getCurrentPosition();
        double currentTime = timer.seconds();
        int deltaTicks = currentPos - lastPos;
        double deltaTime = currentTime - lastTime;
        if (deltaTime < 0.05) return 0;
        if (deltaTime <= 0) return 0;
        double rev = (double) deltaTicks/ticksPerRev;
        double RPM = (rev/deltaTime) * 60;
        //telemetry.addData("Launch", "dT = %4.3f dTick %4d\n",deltaTime,deltaTicks);
        lastPos = currentPos;
        lastTime = currentTime;
        return RPM;
    }


    //----------------------------------------------------------------------------------------------e
    // Jaw Methods:
    //----------------------------------------------------------------------------------------------
    public void launch(double power){
        launcher.setPower(power);
    }



    //----------------------------------------------------------------------------------------------
    // isDone Method:
    //----------------------------------------------------------------------------------------------
    @Override
    public boolean isDone() {
        boolean result = false;
        return result;
    }
}