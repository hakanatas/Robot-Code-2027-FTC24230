package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * Field-centric version of BioBuzzStarterbotTeleopMecanum.
 *
 * The drivetrain is driven by Pedro Pathing's Follower, so motor names and directions come from
 * Constants.drivetrainConfig, and the robot heading comes from the Pinpoint localizer.
 * Intake and launcher behave exactly like the robot-centric teleop.
 *
 * Field-centric means pushing the left stick forward always drives the robot away from the driver,
 * no matter which way the robot is facing. The heading is 0 when the OpMode starts, so place the
 * robot facing away from the driver, or press START while it is facing away to reset.
 *
 * This version does not use the Limelight: no auto-aim, R1 shoots as soon as the launcher is up to speed.
 * Use it when the Limelight is unplugged or not in the robot configuration.
 */
@TeleOp(name = "Mec BioBuzz Field Centric (No Limelight)", group = "StarterBot")
public class BioBuzzFieldCentricTeleopNoLimelight extends OpMode {

    private Follower follower;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    // Launcher speed in ticks/s (28 ticks per rev). D-pad right/left changes it by LAUNCHER_STEP.
    // The windmill only feeds once the launcher is within LAUNCHER_FEED_MARGIN of the target.
    public final int DEFAULT_LAUNCHER_VELOCITY = 1450; //3107 RPM
    public final int LAUNCHER_STEP = 10;
    public final int LAUNCHER_FEED_MARGIN = 50;
    public final int MIN_LAUNCHER_VELOCITY = 800;
    public final int MAX_LAUNCHER_VELOCITY = 2600;
    int launcherTargetVelocity = DEFAULT_LAUNCHER_VELOCITY;

    // L1 runs the windmill and the launcher slowly backwards, e.g. to free a stuck element.
    public final double WINDMILL_SLOW_REVERSE_POWER = -0.3;
    public final int LAUNCHER_REVERSE_VELOCITY = -400; // ticks/s

    // Drive speed as a fraction of full power. D-pad up/down changes it by SPEED_STEP.
    public final double DEFAULT_DRIVE_SPEED = 0.7;
    public final double SPEED_STEP = 0.1;
    public final double MIN_DRIVE_SPEED = 0.2;
    public final double MAX_DRIVE_SPEED = 1.0;
    double driveSpeed = DEFAULT_DRIVE_SPEED;

    double intakePower;

    @Override
    public void init() {
        // Creates the drivetrain, Pinpoint localizer and Foresight from pedro/Constants.java
        follower = Constants.create(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        rightIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        // Pinpoint keeps its pose from the previous OpMode (PinpointConfig only recalibrates the IMU),
        // so zero it here: the direction the robot faces at START becomes "forward".
        follower.setPose(new Pose(0, 0, 0));
        follower.update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) {
            driveSpeed = Math.min(MAX_DRIVE_SPEED, driveSpeed + SPEED_STEP);
        }
        if (gamepad1.dpadDownWasPressed()) {
            driveSpeed = Math.max(MIN_DRIVE_SPEED, driveSpeed - SPEED_STEP);
        }

        // Pedro uses +strafe = left and +turn = counterclockwise, so the X sticks are negated
        // (same mapping as Pedro's own Driving test).
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y * driveSpeed,
                -gamepad1.left_stick_x * driveSpeed,
                -gamepad1.right_stick_x * driveSpeed,
                follower.pose().heading()
        );
        follower.manual(powers);

        // Point the robot away from the driver and press START (OPTIONS on PlayStation) to reset heading.
        if (gamepad1.startWasPressed()) {
            follower.setPose(follower.pose().withHeading(0));
        }

        follower.update();

        if (gamepad1.dpadRightWasPressed()) {
            launcherTargetVelocity = Math.min(MAX_LAUNCHER_VELOCITY, launcherTargetVelocity + LAUNCHER_STEP);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            launcherTargetVelocity = Math.max(MIN_LAUNCHER_VELOCITY, launcherTargetVelocity - LAUNCHER_STEP);
        }

        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        launch();

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        Pose pose = follower.pose();
        telemetry.addData("Drive speed", "%.0f%%  (D-pad up/down)", driveSpeed * 100);
        telemetry.addData("Launcher target", "%d  (D-pad left/right)", launcherTargetVelocity);
        telemetry.addData("X (in)", "%.1f", pose.x());
        telemetry.addData("Y (in)", "%.1f", pose.y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
        telemetry.addData("Launcher velocity", "%.0f", launcher.getVelocity());
    }

    void launch() {
        if (gamepad1.right_bumper) {
            launcher.setVelocity(launcherTargetVelocity);
        } else if (gamepad1.left_bumper) {
            launcher.setVelocity(LAUNCHER_REVERSE_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        if (gamepad1.right_bumper && launcher.getVelocity() > launcherTargetVelocity - LAUNCHER_FEED_MARGIN) {
            windmillServo.setPower(1);
            intakePower += 0.5;
        } else if (gamepad1.left_bumper) {
            windmillServo.setPower(WINDMILL_SLOW_REVERSE_POWER);
        } else {
            windmillServo.setPower(0);
        }
    }
}
