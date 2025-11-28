package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class CatHW_Launcher {

    private CatHW_Async mainHardware;
    public DcMotorEx launcher;
    private static final int ticksPerRev = 28;

    private ElapsedTime timer = new ElapsedTime();


    private int lastPos = 0;
    private double lastTime = 0;

    private double rpmEMA = 0.0;
    private double rpmAlpha = 0.2;
    private int accTicks = 0;
    private double accTime = 0.0;
    private final double rpmSamplePeriod = 0.05;

    private ElapsedTime pidTimer = new ElapsedTime();
    private double prevDerivative = 0;
    private double prevOutput = 0;
    private double maxDeltaOutputPerSec = 0.3;
    private double integralLimit = 10000.0;
    private double derivativeFilterAlpha = 0.95;
    private double outputMin = 0.0;

    public double kP = 0.00015;
    public double kI = 0.0;
    public double kD = 0.00005;
    public double kF = 0.000194;

    public double targetRPM;
    private double integral = 0;
    private double lastError = 0;

    public CatHW_Launcher(CatHW_Async mainHardware) {
        this.mainHardware = mainHardware;
    }

    public void init() {
        launcher = mainHardware.hwMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lastPos = launcher.getCurrentPosition();
        lastTime = timer.seconds();
        targetRPM = 0;
        pidTimer.reset();
    }

    public void resetPID() {
        integral = 0;
        lastError = 0;
        pidTimer.reset();
    }

    public double getRPM() {
        int currentPos = launcher.getCurrentPosition();
        double currentTime = timer.seconds();
        int deltaTicks = currentPos - lastPos;
        double deltaTime = currentTime - lastTime;
        if (deltaTime <= 0) return rpmEMA;

        accTicks += deltaTicks;
        accTime += deltaTime;

        double reportedRPM = rpmEMA;
        if (accTime >= rpmSamplePeriod) {
            double rev = (double) accTicks / ticksPerRev;
            double measuredRPM = (rev / accTime) * 60.0;
            if (rpmEMA <= 0.0) rpmEMA = measuredRPM;
            else rpmEMA = rpmAlpha * measuredRPM + (1.0 - rpmAlpha) * rpmEMA;

            accTicks = 0;
            accTime = 0.0;
            reportedRPM = rpmEMA;
        }

        lastPos = currentPos;
        lastTime = currentTime;
        return reportedRPM;
    }

    public double updatePID(double measuredRPM) {
        double dt = Math.max(1e-3, pidTimer.seconds());
        pidTimer.reset();

        double error = targetRPM - measuredRPM;

        integral += error * dt;
        integral = Range.clip(integral, -integralLimit, integralLimit);

        double rawDerivative = (error - lastError) / dt;
        double derivative = derivativeFilterAlpha * prevDerivative + (1.0 - derivativeFilterAlpha) * rawDerivative;
        prevDerivative = derivative;

        lastError = error;

        double out = kP * error + kI * integral + kD * derivative + kF * targetRPM;

        out = Range.clip(out, outputMin, 1.0);

        double maxDelta = maxDeltaOutputPerSec * dt;
        double delta = Range.clip(out - prevOutput, -maxDelta, maxDelta);
        out = prevOutput + delta;
        prevOutput = out;

        return out;
    }

    public void setPowerFromPID() {
        double rpm = getRPM();
        double power = updatePID(rpm);
        launcher.setPower(power);
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        setPowerFromPID();
    }
    public void farLaunch(){
        targetRPM = 2000;
        setPowerFromPID();
    }
}
