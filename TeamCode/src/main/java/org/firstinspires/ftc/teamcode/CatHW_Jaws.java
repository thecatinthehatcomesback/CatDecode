package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
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

    public DcMotor armMotor = null;
    public DcMotor armExtend = null;
    public DcMotor launcher;
   public Servo gripper = null;
    public Servo wrist = null;

    public ElapsedTime liftTime = null;
    public ElapsedTime pidTimer = null;
    public int target;
    //values for pid
    double kP = 0.002;
    double kI = 0.0;
    double kD = 0.0003;
    double feedForword = 0.3;

    double lastError;
    double lastTime;
    private static final int TICKS_PER_REV = 112;


    public Update_PID ourThread = null;
    private static final double ticksPerRev = (3.61*3.61*5.23*28);

    private static final double ticksPerDegree = ticksPerRev/360;
    private static final double startAngle=-12;
    private static final double maxPower= 0.6;
    public static boolean didAuto = false;

    int launchLastPosition;
    double launchLastTime;
    // Timers: //
    private ElapsedTime timer = new ElapsedTime();


    /* Constructor */
    public CatHW_Jaws(CatHW_Async mainHardware) {
        super(mainHardware);

    }


    /* Initialize standard Hardware interfaces */
    public void init() {

        target=0;
        // Define and initialize motors: /armMotor/


        launcher=hwMap.dcMotor.get("launcher");
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //launchLastPosition = launcher.getCurrentPosition();
        //launchLastTime = timer.seconds();




        liftTime = new ElapsedTime();
        pidTimer = new ElapsedTime();



        ourThread = new Update_PID(this);
        ourThread.start();
    }



    //----------------------------------------------------------------------------------------------e
    // Jaw Methods:
    //----------------------------------------------------------------------------------------------
    public void launch(double power){
        launcher.setPower(power);
    }
    /*public double launchRPM() {
        int currentPosition = launcher.getCurrentPosition();
        double currentTime = timer.seconds();

        int deltaTicks = currentPosition - launchLastPosition;
        double deltaTime = currentTime - launchLastTime;

        if (deltaTime <= 0) return 0;

        double revolutions = (double) deltaTicks / TICKS_PER_REV;
        double rpm = (revolutions / deltaTime) * 60.0;

        // Update for next loop
        launchLastPosition = currentPosition;
        launchLastTime = currentTime;

        return rpm;
    }*/

    public void updatePID(){
        int current = armMotor.getCurrentPosition();
        double error = target-current;
        double angle = (current / ticksPerDegree)+startAngle;
        double derivative = (error - lastError) / (pidTimer.seconds()-lastTime);
        double power = 0;
        if (target/ticksPerDegree > 0) {
            power = error * kP + derivative * kD + Math.cos(Math.toRadians(angle)) * feedForword;
        } else {
            power = error * kP + derivative * kD ;
        }
        power = Math.min(power,maxPower);
        power = Math.max(power,-maxPower);
        armMotor.setPower(power);
        Log.d("catbot",String.format("arm err %.3f angle %.3f power %.3f der %.3f",error,angle,power,derivative));
        lastError=error;
        lastTime = pidTimer.seconds();

        if (Math.abs(armExtend.getCurrentPosition()-armExtend.getTargetPosition())<20){
          armExtend.setPower(0);
        }
        //limit extension distence
        if ((angle<45) && (armExtend.getTargetPosition()>2625)){
            armExtend.setTargetPosition(2100);
        }
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