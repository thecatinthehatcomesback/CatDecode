package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CatLift extends CatHW_Subsystem {
    public CatLift(CatHW_Async mainHardware){
        super(mainHardware);
    }
    public DcMotor liftMotor = null;

    boolean isDone;

    public void init(){

        liftMotor = hwMap.dcMotor.get("lift");

        // Define motor directions: //
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
    }

   public  void  goUp(){
        liftMotor.setPower(1);
   }

    public  void  stopLift(){
        liftMotor.setPower(0);
    }
}
