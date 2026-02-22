package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name="RedClose_Fixed", group="A")
public class RedClose extends LinearOpMode {

    // ---------------- Hardware (FIELDS, NOT LOCAL) ----------------
    private DcMotorEx rightFly, leftFly;
    private DcMotorEx intake;        // your intake motor
    private Servo hood, gate;        // your hood + gate servo
    private Limelight3A limelight;

    // ---------------- Pedro ----------------
    private Follower follower;

    // ---------------- Tunables ----------------
    // Flywheel velocity control (same idea as your teleop)
    public static double kP = 0.0001;
    public static double kV = 0.0;
    public static double kS = 0.0;

    public static double TARGET_VEL = 2000;         // ticks/sec (tune)
    public static double VEL_TOL = 120;             // ticks/sec tolerance
    public static long   SPINUP_TIMEOUT_MS = 1200;

    // Feeding / gate timing
    public static double GATE_CLOSED = 0.15;
    public static double GATE_OPEN   = 0.03;
    public static long   GATE_OPEN_MS = 120;
    public static long   GATE_CLOSE_MS = 140;

    // Intake power during pickup
    public static double INTAKE_PWR = 0.8;

    // Hood positions (optional)
    public static double HOOD_SHOOT_POS = 0.55;

    // ---------------- Poses (YOU MUST KEEP YOUR OWN) ----------------
    // NOTE: I don’t know your exact poses from your file upload name,
    // so keep YOUR existing Pose constants here exactly as you had them.
    // Example placeholders:
    private static final Pose START = new Pose(-72, 72, Math.toRadians(180));

    private static final Pose FIRST_SHOT_POSE   = new Pose(-25, 20, Math.toRadians(130));
    private static final Pose BACK_SHOT_POSE    = new Pose(-28, 31, Math.toRadians(120));

    // If you want a different shoot spot for row 3 later, change this pose.
    private static final Pose THIRD_SHOT_POSE = new Pose(-28, 31, Math.toRadians(120));

    // LINEUP POSES
    private static final Pose FIRST_SHOT_LINEUP  = new Pose(5, 15, Math.toRadians(90));
    private static final Pose SECOND_SHOT_LINEUP = new Pose(40, 32, Math.toRadians(90));
    private static final Pose THIRD_SHOT_LINEUP  = new Pose(75, 32, Math.toRadians(90));

    private static final Pose INTAKE_ROW1_POSE  = new Pose(5, 95, Math.toRadians(90));
    private static final Pose ROW2_START_POSE   = new Pose(42, 32, Math.toRadians(90));
    private static final Pose INTAKE_ROW2_POSE  = new Pose(42, 110, Math.toRadians(90));

    private static final Pose ROW3_START_POSE   = new Pose(78, 32, Math.toRadians(90));
    private static final Pose INTAKE_ROW3_POSE  = new Pose(78, 115, Math.toRadians(90));

    private static final Pose FINAL_SHOT_POSE   = new Pose(-27, 37, Math.toRadians(120));

    @Override
    public void runOpMode() {

        // ---------------- Hardware init ----------------
        initHardware();

        // ---------------- Pedro init ----------------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START);

        // ---------------- Build Paths (keep your structure) ----------------
        PathChain toFirstShot   = line(START, FIRST_SHOT_POSE);
        PathChain toBackShot    = line(FIRST_SHOT_POSE, BACK_SHOT_POSE);

        PathChain toFirstLineup = line(BACK_SHOT_POSE, FIRST_SHOT_LINEUP);
        PathChain lineupToRow1  = line(FIRST_SHOT_LINEUP, INTAKE_ROW1_POSE);
        PathChain backToShoot1  = line(INTAKE_ROW1_POSE, BACK_SHOT_POSE);

        PathChain toRow2Start   = line(BACK_SHOT_POSE, ROW2_START_POSE);
        PathChain toSecondLineup= line(ROW2_START_POSE, SECOND_SHOT_LINEUP);
        PathChain lineupToRow2  = line(SECOND_SHOT_LINEUP, INTAKE_ROW2_POSE);
        PathChain backToShoot2  = line(INTAKE_ROW2_POSE, BACK_SHOT_POSE);

        PathChain toRow3Start   = line(THIRD_SHOT_POSE, ROW3_START_POSE);
        PathChain toThirdLineup = line(ROW3_START_POSE, THIRD_SHOT_LINEUP);
        PathChain lineupToRow3  = line(THIRD_SHOT_LINEUP, INTAKE_ROW3_POSE);

        PathChain toFinalShot   = line(INTAKE_ROW3_POSE, FINAL_SHOT_POSE);

        telemetry.addLine("RedClose_Fixed ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---------------- AUTO SEQUENCE ----------------
        followAndWait(toFirstShot);
        shootBurst3();

        followAndWait(toBackShot);

        followAndWait(toFirstLineup);
        setIntake(true);
        followAndWait(lineupToRow1);
        setIntake(false);

        followAndWait(backToShoot1);
        shootBurst3();

        followAndWait(toRow2Start);
        followAndWait(toSecondLineup);
        setIntake(true);
        followAndWait(lineupToRow2);
        setIntake(false);

        followAndWait(backToShoot2);
        shootBurst3();

        followAndWait(toRow3Start);
        followAndWait(toThirdLineup);
        setIntake(true);
        followAndWait(lineupToRow3);
        setIntake(false);

        followAndWait(toFinalShot);
        shootBurst3();

        safeStopAll();
    }

    // ---------------- Hardware init ----------------
    private void initHardware() {
        // Flywheels (YOUR TELEOP NAMES)
        rightFly = hardwareMap.get(DcMotorEx.class, "rightmotor");
        leftFly  = hardwareMap.get(DcMotorEx.class, "leftmotor");

        // Match your teleop mirrored setup (change if your physical build differs)
        rightFly.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFly.setDirection(DcMotorSimple.Direction.FORWARD);

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        hood = hardwareMap.get(Servo.class, "hood");
        gate = hardwareMap.get(Servo.class, "gate");

        // Default positions
        gate.setPosition(GATE_CLOSED);
        hood.setPosition(HOOD_SHOOT_POS);

        // Limelight (optional for later aiming logic)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    // ---------------- Path helpers ----------------
    private PathChain line(Pose a, Pose b) {
        return follower.pathBuilder()
                .addPath(new BezierLine(a, b))
                .setLinearHeadingInterpolation(a.getHeading(), b.getHeading())
                .build();
    }

    private void followAndWait(PathChain path) {
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.update();
            idle();
        }
    }

    // ---------------- Shooter ----------------
    private void shootBurst3() {
        // Hood for shooting
        hood.setPosition(HOOD_SHOOT_POS);

        // Spin up flywheel with velocity loop until close enough or timeout
        ElapsedTime t = new ElapsedTime();
        t.reset();

        while (opModeIsActive() && t.milliseconds() < SPINUP_TIMEOUT_MS) {
            double v = Math.abs(rightFly.getVelocity());
            double power = flywheelPowerFor(TARGET_VEL, v);
            rightFly.setPower(power);
            leftFly.setPower(power);

            telemetry.addData("SpinupTarget", TARGET_VEL);
            telemetry.addData("SpinupVel", v);
            telemetry.addData("SpinupPower", power);
            telemetry.update();

            if (Math.abs(TARGET_VEL - v) <= VEL_TOL) break;
            idle();
        }

        // Fire 3 shots by pulsing the gate
        for (int i = 0; i < 3 && opModeIsActive(); i++) {
            gate.setPosition(GATE_OPEN);
            sleep(GATE_OPEN_MS);

            gate.setPosition(GATE_CLOSED);
            sleep(GATE_CLOSE_MS);
        }

        // Keep flywheel running a tiny bit for stability
        sleep(120);
    }

    private double flywheelPowerFor(double targetVel, double measuredVel) {
        double error = targetVel - measuredVel;
        double feedback = kP * error;
        double ff = (targetVel > 0) ? (kV * targetVel + kS) : 0.0;
        return Range.clip(ff + feedback, 0.0, 1.0);
    }

    // ---------------- Intake helpers ----------------
    private void setIntake(boolean on) {
        intake.setPower(on ? INTAKE_PWR : 0.0);
    }

    private void safeStopAll() {
        rightFly.setPower(0);
        leftFly.setPower(0);
        intake.setPower(0);
        gate.setPosition(GATE_CLOSED);
    }
}