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
    public Servo gate = null;
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

        gate = hwMap.servo.get("gate");

        gate.setPosition(0.5);

        intake=hwMap.dcMotor.get("intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transfer = hwMap.dcMotor.get("transfer");
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftTime = new ElapsedTime();
        pidTimer = new ElapsedTime();
    }

    public void transfer(double speed){
        transfer.setPower(speed);
    }
    public void gateOpen(){
        gate.setPosition(0.47);
    }
    public void gateClosed(){
        gate.setPosition(0.60);
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