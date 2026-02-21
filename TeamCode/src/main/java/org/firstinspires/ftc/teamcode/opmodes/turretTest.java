package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.TurretMechanism;
@Configurable
@TeleOp(name="Tune Turret PD")
public class turretTest extends OpMode {
    DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private TurretMechanism turret = new TurretMechanism();

    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 2;

    private boolean lastB = false;
    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastLeft = false;
    private boolean lastRight = false;

    @Override
    public void init() {
        turret.init(hardwareMap, telemetry);
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void start() {
        turret.resetTimer();
    }

    @Override
    public void loop() {


        if (gamepad1.b && !lastB) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpad_left && !lastLeft) {
            turret.setP(turret.getKP() - stepSizes[stepIndex]);
        }
        if (gamepad1.dpad_right && !lastRight) {
            turret.setP(turret.getKP() + stepSizes[stepIndex]);
        }

        if (gamepad1.dpad_up && !lastUp) {
            turret.setD(turret.getKD() + stepSizes[stepIndex]);
        }
        if (gamepad1.dpad_down && !lastDown) {
            turret.setD(turret.getKD() - stepSizes[stepIndex]);
        }
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;
        lastB = gamepad1.b;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;

        turret.update(true);

        // Telemetry
        telemetry.addData("Status", "Tuning Mode");
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.addLine("-------------------");
        telemetry.addData("P Gain", "%.5f", turret.getKP());
        telemetry.addData("D Gain", "%.5f", turret.getKD());
        telemetry.update();
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;


        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
}