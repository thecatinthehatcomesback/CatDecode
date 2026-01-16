package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.atomic.AtomicBoolean;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

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

    private double adjust = 0;
    private void autoAim(double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {

            LLResult result = limelight.getLatestResult();
            double xAngle = 0;
            boolean hasTarget = false;

            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                    if (fr.getFiducialId() == 20 || fr.getFiducialId() == 24) {
                        xAngle = fr.getTargetXDegrees();
                        hasTarget = true;
                        break;
                    }
                }
            }

            if (!hasTarget) {
                // No target → stop rotating
                robot.prowl.drive(0, 0, 0, 0);
                continue;
            }

            // Aligned
            if (Math.abs(xAngle) < 1) {
                robot.prowl.drive(0, 0, 0, 0);
                return;
            }

            // Rotate toward target
            double kP = 0.04; // proportional gain
            double biasDeg = adjust; // compensate for camera/shooter offset
            double turn = (xAngle - biasDeg) * kP;

            // minimum turn power so small errors still correct
            double minTurn = 0.15;
            if (Math.abs(turn) < minTurn) {
                turn = Math.signum(turn) * minTurn;
            }

            // clamp max turn power
            turn = Math.max(-0.35, Math.min(0.35, turn));

            // apply signed turn directly
            robot.prowl.drive(0, 0, turn, 1);

            sleep(20);
        }

        // Timeout fallback
        robot.prowl.drive(0, 0, 0, 0);
    }
    /* Declare OpMode members. */

    CatHW_Async robot = new CatHW_Async();    // All the hardware classes init here.
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private Limelight3A limelight;
    private ElapsedTime runningTime = new ElapsedTime();

    private AtomicBoolean keepRunningPID = new AtomicBoolean(false);
    private Thread pidThread;


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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        robot.jaws.gateClosed();


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
            if (((gamepad1.dpad_left) && delayTimer.seconds() > 0.5)) {
                // Changes Alliance Sides

                adjust = adjust -0.2;
                delayTimer.reset();
            }
            if (((gamepad1.dpad_right) && delayTimer.seconds() > 0.5)) {
                // Changes Alliance Sides

                adjust = adjust +0.2;
                delayTimer.reset();
            }




            /*
             * Telemetry while waiting for PLAY:
             */
            telemetry.addData("Time Delay ","%.0f  seconds",timeDelay);
            telemetry.addData("Alliance","%s",robot.isRedAlliance ? "red":"blue");
            telemetry.addData("Position", "%s",robot.isCloseStart ? "close":"far");
            telemetry.addData("aimAjust","%.1f",adjust);
            dashboardTelemetry.update();
            telemetry.update();

        }



        /*
         * Runs after hit start:
         * DO STUFF FOR the OPMODE!!!
         */

        keepRunningPID.set(true);
        pidThread = new Thread(() -> {
            while (opModeIsActive() && keepRunningPID.get()) {
                robot.launch.setPowerFromPID();
                try { Thread.sleep(20); } catch (InterruptedException e) { break; }
            }
        });
        pidThread.start();


        robot.robotWait(timeDelay);

        if (robot.isRedAlliance) {
            if (robot.isCloseStart) {
                 closeRed();
            } else {
                farRed();
            }
        } else {
            if (robot.isCloseStart) {
                 closeBlue();
            } else {
                farBlue();
            }
        }

        keepRunningPID.set(false);
        try { pidThread.join(); } catch (InterruptedException e) {}

    }


    private void shoot(){
        int i;
        robot.jaws.gateOpen();
        robot.jaws.intake.setPower(1);
       // for(i = 0;i < 3;i++){
           // robot.jaws.transfer(.4);
            //robot.robotWait(1.3);
            //robot.jaws.transfer(0);
            //robot.robotWait(.5);
        //}
        robot.jaws.transfer (0.4);
        robot.robotWait(2.5);
        robot.jaws.intake.setPower(0);
        robot.jaws.transfer(0);
        robot.jaws.gateClosed();
    }
    private void farRed(){
        robot.launch.setTargetRPM(5000);
        robot.prowl.driveto(3,8,-20,0.4,5);
        autoAim(3);
        robot.robotWait(4);
        shoot();
        //go get 1 stack
        robot.prowl.driveto(12,28,-90,0.6,5);
        robot.jaws.intake.setPower(1);
        robot.jaws.transfer(.1);
        robot.prowl.driveto(48,28,-90,.2,5);
        robot.jaws.transfer(0);
        robot.jaws.intake.setPower(0);
        robot.prowl.driveto(7,15,-20,0.6,5);
        autoAim(3);
        robot.robotWait(1);
        shoot();
        //get second stack
        robot.prowl.driveto(12,50,-90,0.6,5);
        robot.jaws.intake.setPower(1);
        robot.jaws.transfer(.1);
        robot.prowl.driveto(48,50,-90,.2,5);
        robot.jaws.transfer(0);
        robot.jaws.intake.setPower(0);
        robot.prowl.driveto(7,15,-20,0.6,5);
        autoAim(3);
        robot.robotWait(1);
        shoot();
        robot.prowl.driveto(7,20,-22,0.6,5);

    }
    private void farBlue() {
        robot.launch.setTargetRPM(3700);
        robot.prowl.driveto(-3,8,24,0.4,5);
        robot.robotWait(1);
        shoot();

        //go get 1 stack
        robot.prowl.driveto(-12,26,90,0.6,5);
        robot.jaws.intake.setPower(1);
        robot.jaws.transfer(.1);
        robot.prowl.driveto(-50,26,90,.2,5);
        robot.jaws.transfer(0);
        robot.prowl.driveto(-7,15,23,0.6,5);
        shoot();
        //get second stack
        robot.prowl.driveto(-12,50,90,0.6,5);
        robot.jaws.intake.setPower(1);
        robot.jaws.transfer(.1);
        robot.prowl.driveto(-50,50,90,.2,5);
        robot.jaws.transfer(0);
        robot.prowl.driveto(-7,15,27,0.6,5);
        shoot();
        robot.prowl.driveto(-7,20,27,0.6,5);
    }
    private void closeRed(){
        robot.launch.setTargetRPM(3000);
        robot.jaws.gateClosed();
        robot.prowl.driveto(25,-38,46,0.6,5);
        robot.jaws.gateOpen();
        autoAim(3);
        robot.robotWait(1);
        shoot();
        robot.robotWait(.2);
        robot.jaws.gateClosed();
        //get first stack
        robot.jaws.intake.setPower(1);
        robot.prowl.driveto(33,-30,0,0.6,5);
        //robot.jaws.transfer();
        robot.prowl.driveto(29,-3 ,0,0.4,5);
        robot.jaws.intake.setPower(0);
        //robot.jaws.transfer(0);
        robot.prowl.driveto(25,-38,45,0.6,5);
        autoAim(3);
        robot.jaws.gateOpen();
        robot.robotWait(1);
        shoot();
        robot.robotWait(.2);
        robot.jaws.gateClosed();
        robot.prowl.driveto(48,-38,0,0.6,5);
        //robot.jaws.transfer(.1);
        robot.jaws.intake.setPower(1);
        robot.prowl.driveto(48,-4 ,0,0.4,5);
        robot.jaws.intake.setPower(0);
        //robot.jaws.transfer(0);
        robot.prowl.driveto(25,-38,45,0.6,5);
        autoAim(3);
        robot.jaws.gateOpen();
        robot.robotWait(1);
        shoot();
        robot.robotWait(.2);
        robot.jaws.gateClosed();
        robot.prowl.driveto(60, -38,0,0.6,5);
        //robot.jaws.transfer(.1);
        robot.jaws.intake.setPower(1);
        robot.prowl.driveto(60,-4 ,0,0.4,5);
        robot.jaws.intake.setPower(0);
        //shoot 2nd
        robot.prowl.driveto(25,-38,45,0.6,5);
        autoAim(3);
        robot.jaws.gateOpen();
        robot.robotWait(1);
        shoot();

//sam was here


    }
    private void closeBlue(){
        robot.launch.setTargetRPM(3000);
        robot.prowl.driveto(-25,-38,-43,0.5,5);
        autoAim(3);
        robot.robotWait(1);
        shoot();
        robot.robotWait(1);
        //get first stack
        robot.jaws.intake.setPower(1);
        robot.prowl.driveto(-33,-30,0,0.5,5);
        robot.jaws.transfer(.1);
        robot.prowl.driveto(-33,-3 ,0,0.3,5);
        robot.jaws.intake.setPower(0);
        robot.jaws.transfer(0);
        robot.prowl.driveto(-25,-38,-41,0.5,5);
        autoAim(3);
        robot.robotWait(1);
        shoot();
        robot.robotWait(1);
        robot.prowl.driveto(-58,-38,0,0.5,5);
        robot.jaws.transfer(.1);
        robot.jaws.intake.setPower(1);
        robot.prowl.driveto(-58,-4 ,0,0.3,5);
        robot.jaws.intake.setPower(0);
        robot.jaws.transfer(0);
        //get second stack
    }
}
