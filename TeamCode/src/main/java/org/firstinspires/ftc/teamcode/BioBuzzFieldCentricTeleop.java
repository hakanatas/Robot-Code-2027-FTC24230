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

import java.util.Locale;

import org.firstinspires.ftc.teamcode.lib.Constants.ShootingParams;
import org.firstinspires.ftc.teamcode.lib.math.InterpolatingDouble;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Superstructure;

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
 * Holding R1 (shoot) spins up the launcher and turns the robot onto the AprilTag: the Limelight gives
 * the target heading and the Pinpoint turns the robot to it, and keeps it on the tag while you drive
 * with the left stick (±30° sweep until the tag is first seen). The windmill only feeds while the robot
 * is aligned (tag visible, |tx| within tolerance, robot turning with the target) AND the launcher is up
 * to speed. Aim gains are tunable live in Panels > Superstructure.
 */
@TeleOp(name = "Mec BioBuzz Field Centric", group = "StarterBot")
public class BioBuzzFieldCentricTeleop extends OpMode {

    private Follower follower;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    // Launcher speed in ticks/s (28 ticks per rev). It comes from the tag distance table
    // (lib/Constants.ShootingParams.kLauncherVelocityValues); D-pad right/left adds a trim of LAUNCHER_STEP.
    // DEFAULT_LAUNCHER_VELOCITY is used until a tag distance has been read.
    // The windmill only feeds once the launcher is within LAUNCHER_FEED_MARGIN of the target.
    public final int DEFAULT_LAUNCHER_VELOCITY = 1220;
    public final int LAUNCHER_STEP = 10;
    public final int LAUNCHER_FEED_MARGIN = 50;
    public final int MIN_LAUNCHER_VELOCITY = 800;
    public final int MAX_LAUNCHER_VELOCITY = 2600;
    int launcherTargetVelocity = DEFAULT_LAUNCHER_VELOCITY;
    double tableLauncherVelocity = DEFAULT_LAUNCHER_VELOCITY;
    int launcherTrim = 0;
    boolean launcherReady = false;

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

    // True while R1 is held and the robot is on the tag right now.
    boolean aligned = false;
    // Set when the first shot starts (aligned + launcher up to speed), cleared when R1 is released.
    // While shooting the heading and launcher speed are frozen, so the robot does not re-align mid-burst.
    boolean shooting = false;

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

        // Limelight ("limelight" in the robot configuration) + tag aim
        Superstructure.init(hardwareMap);
        Superstructure.reset();
        Superstructure.vision.LL3.pipelineSwitch(0);

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
        Superstructure.read();
        Superstructure.periodic();

        if (gamepad1.dpadUpWasPressed()) {
            driveSpeed = Math.min(MAX_DRIVE_SPEED, driveSpeed + SPEED_STEP);
        }
        if (gamepad1.dpadDownWasPressed()) {
            driveSpeed = Math.max(MIN_DRIVE_SPEED, driveSpeed - SPEED_STEP);
        }

        // Pedro uses +strafe = left and +turn = counterclockwise, so the X sticks are negated
        // (same mapping as Pedro's own Driving test).
        double turnInput = -gamepad1.right_stick_x * driveSpeed;

        // R1 held: the Limelight takes over rotation, the left stick still translates.
        // If the tag drops out after it was seen, the robot holds the last target heading.
        if (gamepad1.right_bumper) {
            turnInput = Superstructure.updateTargetLockOrSweep(
                    follower.pose().heading(), follower.velocity().omega,
                    Math.hypot(follower.velocity().vx, follower.velocity().vy));
        } else {
            // Reset so the next R1 press starts with a fresh target and sweep.
            Superstructure.stopAim();
        }

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y * driveSpeed,
                -gamepad1.left_stick_x * driveSpeed,
                turnInput,
                follower.pose().heading()
        );
        follower.manual(powers);

        // Point the robot away from the driver and press START (OPTIONS on PlayStation) to reset heading.
        if (gamepad1.startWasPressed()) {
            follower.setPose(follower.pose().withHeading(0));
            // The aim target and heading history were in the old heading frame
            Superstructure.stopAim();
        }

        follower.update();

        if (gamepad1.dpadRightWasPressed()) {
            launcherTrim += LAUNCHER_STEP;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            launcherTrim -= LAUNCHER_STEP;
        }

        // Launcher speed from the tag distance table, frozen while shooting
        double tagDistance = Superstructure.getTagDistanceCm();
        if (!shooting && tagDistance > 0) {
            tableLauncherVelocity = ShootingParams.kLauncherVelocityMap
                    .getInterpolated(new InterpolatingDouble(tagDistance)).value;
        }
        launcherTargetVelocity = (int) Math.max(MIN_LAUNCHER_VELOCITY,
                Math.min(MAX_LAUNCHER_VELOCITY, Math.round(tableLauncherVelocity) + launcherTrim));

        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        aligned = gamepad1.right_bumper && Superstructure.isAligned(follower.velocity().omega);
        launcherReady = launcher.getVelocity() > launcherTargetVelocity - LAUNCHER_FEED_MARGIN;
        if (gamepad1.right_bumper && !shooting && aligned && launcherReady) {
            // First shot: lock the heading, no more re-aligning until R1 is released
            shooting = true;
            Superstructure.freezeAim(follower.pose().heading());
        }
        if (!gamepad1.right_bumper) {
            shooting = false;
        }

        launch();

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        Superstructure.write();

        Pose pose = follower.pose();

        double tagDistanceCm = Superstructure.getTagDistanceCm();
        telemetry.addData("Limelight sees tags", "%s  (targets: %s)", Superstructure.vision.allTagIds,
                Superstructure.TARGET_TAG_GROUPS);
        telemetry.addData("Tag distance", !Superstructure.vision.tv ? "no target tag"
                : tagDistanceCm <= 0 ? "no 3D data: turn on Full 3D (Limelight > AprilTag pipeline > Advanced)"
                : String.format(Locale.US, "%.1f cm  (%.1f in)", tagDistanceCm, tagDistanceCm / 2.54));
        telemetry.addData("Launcher target", "%d  (table %.0f, trim %+d: D-pad left/right)",
                launcherTargetVelocity, tableLauncherVelocity, launcherTrim);
        telemetry.addData("Box view angle", Superstructure.vision.boxOffsetApplied
                ? String.format(Locale.US, "%.0f°  (16 cm box offset on)", Superstructure.vision.boxViewAngleDeg)
                : "box offset off (needs 2+ tags with 3D)");
        telemetry.addData("Tags / ty", "%s / %.1f°  (aiming at the middle)", Superstructure.vision.tagIds,
                Superstructure.vision.ty);
        telemetry.addData("Aim", !gamepad1.right_bumper ? "MANUAL"
                : shooting ? "SHOOTING (heading locked)"
                : aligned ? "ALIGNED"
                : Superstructure.hasAimTarget() ? "ALIGNING" : "SWEEPING (±30°)");
        telemetry.addData("Tag visible", Superstructure.vision.tv);
        telemetry.addData("tx robot / camera (deg)", "%.1f / %.1f", Superstructure.vision.tx,
                Superstructure.vision.txCamera);
        telemetry.addData("Aim turn power", "%.2f", turnInput);
        telemetry.addData("Turn speed (deg/s)", "%.0f", Math.toDegrees(follower.velocity().omega));
        telemetry.addData("Target drift (deg/s)", "%.0f", Math.toDegrees(Superstructure.getTargetRate()));
        telemetry.addData("Drive speed", "%.0f%%  (D-pad up/down)", driveSpeed * 100);
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

        // Only feed once the robot was aligned when shooting started, and while the launcher is up to speed.
        if (shooting && launcherReady) {
            windmillServo.setPower(1);
            intakePower += 0.5;
        } else if (gamepad1.left_bumper) {
            windmillServo.setPower(WINDMILL_SLOW_REVERSE_POWER);
        } else {
            windmillServo.setPower(0);
        }
    }
}
