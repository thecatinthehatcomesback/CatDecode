package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * MainAutonomous.java
 *
 *
 * A Linear OpMode class to be an autonomous method for both Blue & Red alliance sides where we pick
 * which side of the alliance bridge we start off at with gamepad1 as well as selecting alliance
 * color and whether we park under the alliance bridge closer or further from the game field wall.
 * Also will detect the position and deliver the skystone using machine vision and move the
 * foundation.
 *
 * Mec_Odo_AutonomousLevel6_Statey is written to use machine vision and SkyStone delivery to our
 * autonomous route with the help intake jaws that suck in a stone at any orientation using a
 * "touch it-own it" approach.  A servo and two motors make up TC-73/Bucky's arm and stack stones as
 * well as our team marker.

 * This autonomous is used for our State Championship(February 7-8, 2020).
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back.
 */

@Autonomous(name="New Auto", group="CatAuto")

public class MainAuto extends LinearOpMode {

    /* Declare OpMode members. */

    CatHW_Async robot = new CatHW_Async();    // All the hardware classes init here.
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;

    private ElapsedTime runningTime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        /*
        Initialize the setDrivePowers system variables.  The init() methods of our hardware class
        does all the work:
         */
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        robot.init(hardwareMap, this);
        robot.prowl.telemetry = telemetry;


        /*
        Init Delay Option Select:
         */

        // After init is pushed but before Start we can change the delay using dpad up/down //
        delayTimer.reset();
        // Runs a loop to change certain settings while we wait to start
        while (!opModeIsActive() && !isStopRequested()) {
            if (this.isStopRequested()) {
                // Leave the loop if STOP is pressed
                return;
            }
            if (gamepad1.dpad_up && (delayTimer.seconds() > 0.8)) {
                // Increases the amount of time we wait
                timeDelay += 1;
                delayTimer.reset();
            }
            if (gamepad1.dpad_down && (delayTimer.seconds() > 0.8)) {
                // Decreases the amount of time we wait
                if (timeDelay > 0) {
                    // No such thing as negative time
                    timeDelay -= 1;
                }
                delayTimer.reset();
            }
            if (((gamepad1.circle) && delayTimer.seconds() > 0.5)) {
                // Changes Alliance Sides
                if (robot.isRedAlliance) {
                    robot.isRedAlliance = false;
                } else {
                    robot.isRedAlliance = true;
                }
                delayTimer.reset();
            }

            if (((gamepad1.square) && delayTimer.seconds() > 0.5)) {
                // Changes Alliance Sides
                if (robot.isCloseStart) {
                    robot.isCloseStart = false;
                } else {
                    robot.isCloseStart = true;
                }
                delayTimer.reset();
            }




            /*
             * Telemetry while waiting for PLAY:
             */
            telemetry.addData("Time Delay ","%.0f  seconds",timeDelay);
            telemetry.addData("Alliance","%s",robot.isRedAlliance ? "red":"blue");
            telemetry.addData("Position", "%s",robot.isCloseStart ? "close":"far");
            dashboardTelemetry.update();
            telemetry.update();

        }



        /*
         * Runs after hit start:
         * DO STUFF FOR the OPMODE!!!
         */

        robot.robotWait(timeDelay);
        if (robot.isRedAlliance) {
            if (robot.isCloseStart) {
                // closeRed();
            } else {
                farRed();
            }
        } else {
            if (robot.isCloseStart) {
                // closeBlue();
            } else {
                farBlue();
            }
        }

        if(isStopRequested()) return;





    }
    private void farRed(){
        robot.launch.setTargetRPM(2100);
        robot.prowl.driveto(3,10,-27,0.4,5);
        robot.robotWait(1);
        robot.jaws.transfer(.3);
        robot.jaws.intake.setPower(1);
        robot.robotWait(5);
        robot.jaws.intake.setPower(0);
        robot.jaws.transfer(0);
        //go get 1 stack
        robot.prowl.driveto(5,26,-90,0.4,5);
        robot.jaws.intake.setPower(1);
        robot.jaws.transfer(.2);
        robot.prowl.driveto(50,26,-90,.2,5);
        robot.jaws.transfer(0);
        robot.robotWait(.5);
        robot.prowl.driveto(7,15,-27,0.4,5);
        robot.launch.setTargetRPM(2200);
        robot.robotWait(2);
        robot.jaws.transfer(.3);
        robot.robotWait(5);
        //get second stack
        robot.prowl.driveto(5,50,-90,0.4,5);
        robot.jaws.intake.setPower(1);
        robot.jaws.transfer(.1);
        robot.prowl.driveto(50,50,-90,.2,5);
        robot.jaws.transfer(0);
        robot.robotWait(.5);
        robot.prowl.driveto(7,15,-27,0.4,5);
        robot.robotWait(.5);
        robot.jaws.transfer(.3);
        robot.robotWait(5);
    }
    private void farBlue() {
        robot.launch.setTargetRPM(2000);
        robot.prowl.driveto(-3,10,27,0.4,5);
        robot.robotWait(1);
        robot.jaws.transfer(.3);
        robot.jaws.intake.setPower(1);
        robot.robotWait(5);
        robot.jaws.intake.setPower(0);
        robot.jaws.transfer(0);

        //go get 1 stack
        robot.prowl.driveto(-5,26,90,0.4,5);
        robot.jaws.intake.setPower(1);
        robot.jaws.transfer(.3);
        robot.prowl.driveto(-50,26,90,.2,5);
        robot.jaws.transfer(0);
        robot.robotWait(.5);
        robot.prowl.driveto(-7,15,27,0.4,5);
        robot.robotWait(.5);
        robot.jaws.transfer(.3);
        robot.robotWait(5);
        //go get stack 2
        robot.prowl.driveto(-5,50,90,0.4,5);
        robot.jaws.intake.setPower(1);
        robot.jaws.transfer(.3);
        robot.prowl.driveto(-50,50,90,.2,5);
        robot.jaws.transfer(0);
        robot.robotWait(.5);
        robot.prowl.driveto(-7,15,27,0.4,5);
        robot.robotWait(.5);
        robot.jaws.transfer(.3);
        robot.robotWait(5);

    }
}

