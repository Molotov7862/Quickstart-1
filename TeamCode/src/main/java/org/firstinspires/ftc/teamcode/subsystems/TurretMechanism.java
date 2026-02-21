package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Configurable
public class TurretMechanism {

    private DcMotorEx turret;
    private Limelight3A limelight;

    public static double kP = 0.0343;
    public static double kD = 0.00001;

    private double lastError = 0;
    private double angleTolerance = 0.2;
    private final double MAX_POWER = 0.8;

    private double power = 0;

    private final ElapsedTime timer = new ElapsedTime();


    public static int leftLimit  = -600;
    public static int rightLimit =  600;


    public void init(HardwareMap hwMap, Telemetry telemetry) {
        limelight.start();

        turret = hwMap.get(DcMotorEx.class, "turret");

        // Reset encoder
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hwMap.get(Limelight3A.class, "limelight");
        telemetry.addData("Version 1.0", "Turret Rotater!");
        telemetry.addData("turretPos", pos);
        telemetry.addData("turretPwr", power);
        telemetry.addData("tx", limelight.getLatestResult().getTx());
        limelight.pipelineSwitch(0);


        timer.reset();
    }
    int pos = turret.getCurrentPosition();

    public void setP(double newKP) {
        kP = newKP;
    }

    public double getKP() {
        return kP;
    }

    public void setD(double newKD) {
        kD = newKD;
    }

    public double getKD() {
        return kD;
    }

    public void resetTimer() {
        timer.reset();
    }

    public void update(boolean isAligning) {

        // If not aligning, stop turret
        if (!isAligning) {
            turret.setPower(0);
            lastError = 0;
            timer.reset();
            return;
        }

        LLResult llResult = limelight.getLatestResult();

        double deltaTime = timer.seconds();
        timer.reset();

        // If no valid AprilTag detected
        if (llResult == null || !llResult.isValid()) {
            turret.setPower(0);
            lastError = 0;
            return;
        }

        double error = llResult.getTx(); // Horizontal offset (degrees)

        // If within tolerance, stop
        if (Math.abs(error) < angleTolerance) {
            turret.setPower(0);
            lastError = error;
            return;
        }

        // Proportional term
        double pTerm = error * kP;

        // Derivative term
        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }

        power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);


        // hard limits
        if (pos >= rightLimit && power > 0) power = 0;
        if (pos <= leftLimit  && power < 0) power = 0;

        turret.setPower(power);


        turret.setPower(power);

        lastError = error;
    }
}
