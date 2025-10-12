package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CatProwl extends CatHW_Subsystem {
    public CatProwl(CatHW_Async mainHardware){
        super(mainHardware);
    }
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;

    boolean isDone;

    SparkFunOTOS myOtos;
    Pose2d targetPos;
    SparkFunOTOS.Pose2D lastPos;
    double driveSpeed;
    int wrap;

    public void init(){

        leftFrontMotor = hwMap.dcMotor.get("leftFront");
        rightFrontMotor = hwMap.dcMotor.get("rightFront");
        leftRearMotor = hwMap.dcMotor.get("leftRear");
        rightRearMotor = hwMap.dcMotor.get("rightRear");

        // Define motor directions: //
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);

        myOtos = hwMap.get(SparkFunOTOS.class, "sensor_otos");
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        configureOtos();
        lastPos = new SparkFunOTOS.Pose2D(0,0,0);
        wrap=0;
    }
    private void configureOtos() {
        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

    }
    public void driveto(double x,double y,double theta,double speed,double timeout){
        targetPos = new Pose2d(x,y,Math.toRadians(theta));
        driveSpeed=speed;
        isDone = false;
        ElapsedTime runTime=new ElapsedTime();
        runTime.reset();
        while (!isDone && runTime.seconds()<timeout){
            doDrive();
        }
    }
    public void
    doDrive(){
        double kPdrive=0.3;
        double kIdrive=0.01;
        double kDdrive=0.01;
        double kProt=0.5;
        double kIrot=0.01;
        double kDrot=0.01;


        SparkFunOTOS.Pose2D currentPos = myOtos.getPosition();
        if ((currentPos.h>90)&&(lastPos.h<-90)){
            wrap=wrap-1;
        }
        if ((currentPos.h<-90)&&(lastPos.h>90)) {
            wrap = wrap + 1;
        }
        lastPos.x = currentPos.x;
        lastPos.y = currentPos.y;
        lastPos.h = currentPos.h;
        currentPos.h=currentPos.h+(360*wrap);

        double dist = Math.sqrt( Math.pow(targetPos.getX() - currentPos.x , 2) + Math.pow(targetPos.getY() - currentPos.y,2) );
        double motionAngle=Math.atan2(targetPos.getY() - currentPos.y,targetPos.getX() - currentPos.x);
        double rotDiff=targetPos.getHeading()-Math.toRadians(currentPos.h);
        double drivePow=driveSpeed*Math.min(dist*kPdrive,1.0);
        double ypow= Math.sin(motionAngle-Math.toRadians(currentPos.h))*drivePow;
        double xpow= Math.cos(motionAngle-Math.toRadians(currentPos.h))*drivePow;
        double rotErr=rotDiff*kProt;
        double rpow= -Math.max(-1,Math.min(rotErr,1));


        double denominator = Math.max(Math.abs(ypow) + Math.abs(xpow) + Math.abs(rpow), 1);
        double frontLeftPower = (ypow + xpow + rpow) / denominator;
        double backLeftPower = (ypow - xpow + rpow) / denominator;
        double frontRightPower = (ypow - xpow - rpow) / denominator;
        double backRightPower = (ypow + xpow - rpow) / denominator;

        leftFrontMotor.setPower(frontLeftPower);
        leftRearMotor.setPower(backLeftPower);
        rightFrontMotor.setPower(frontRightPower);
        rightRearMotor.setPower(backRightPower);
        Log.d("catbot",String.format("tar %.1f %.1f %.0f cur %.1f %.1f %.0f %.0f wr: %d LF %.2f LR %.2f RF %.2f RR %.2f dist %.2f motionAngle %.2f xpow %.2f ypow %.2f rpow %.2f " ,
                targetPos.getX(), targetPos.getY(), Math.toDegrees(targetPos.getHeading()),currentPos.x , currentPos.y, currentPos.h,lastPos.h, wrap, frontLeftPower, backLeftPower, frontRightPower, backRightPower, dist, motionAngle, xpow, ypow, rpow ));
        if(dist<1.0){
            setDrivePowers(0,0,0,0);
            isDone=true;
        }

    }
    public void driveto(double x,double y,double theta){
        driveto(x,y,theta,0.5,2.0);
    }

    /**
     * Sets powers to the four drive train motors.
     *
     * @param leftFront  motor's power.
     * @param rightFront motor's power.
     * @param leftBack   motor's power.
     * @param rightBack  motor's power.
     */
    public void setDrivePowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        leftFrontMotor.setPower(leftFront);
        rightFrontMotor.setPower(rightFront);
        leftRearMotor.setPower(leftBack);
        rightRearMotor.setPower(rightBack);
    }

    public double findScalor(double leftFrontValue, double rightFrontValue,
                             double leftBackValue, double rightBackValue) {
        /*
        PLANS:
        1: Look at all motor values
        2: Find the highest absolute value (the "scalor")
        3: If the highest value is not more than 1.0, we don't need to change the values
        4: But if it is higher than 1.0, we need to find the scale to get that value down to 1.0
        5: Finally, we pass OUT the scale factor so that we can scale each motor down
         */
        double scalor = 0;
        double scaleFactor;

        double[] values;
        values = new double[4];
        values[0] = Math.abs(leftFrontValue);
        values[1] = Math.abs(rightFrontValue);
        values[2] = Math.abs(leftBackValue);
        values[3] = Math.abs(rightBackValue);

        // Find highest value:
        for (int i = 0; i + 1 < values.length; i++) {
            if (values[i] > scalor) {
                scalor = values[i];
            }
        }

        // If the highest absolute value is over 1.0, we need to get to work!  Otherwise, we done...
        if (scalor > 1.0) {
            // Get the reciprocal:
            scaleFactor = 1.0 / scalor;
        } else {
            // Set to 1 so that we don't change anything we don't have to...
            scaleFactor = 1.0;
        }

        // Now we have the scale factor!
        return scaleFactor;
        // After finding scale factor, we need to scale each motor power down by the same amount...
    }
}
