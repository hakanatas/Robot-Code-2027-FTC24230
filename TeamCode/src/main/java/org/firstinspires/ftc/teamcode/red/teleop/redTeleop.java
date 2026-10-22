package org.firstinspires.ftc.teamcode.red.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.OpModeStorage;
import org.firstinspires.ftc.teamcode.subsystems.Superstructure;

@TeleOp(name = "redSaha")
public class redTeleop extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        if (OpModeStorage.autonomousEndPose != null) {
            follower.setPose(OpModeStorage.autonomousEndPose);
        } else {
            follower.setPose(new Pose(0, 0, 0));
        }

        Superstructure.init(hardwareMap);
        Superstructure.reset();

        if (Superstructure.vision.LL3 != null) {
            Superstructure.vision.LL3.pipelineSwitch(0);
        }
    }

    @Override
    public void loop() {
        follower.update();
        Superstructure.read();
        Superstructure.periodic();

        Pose robotPose = follower.pose();
        if (robotPose == null) return;

        double turnInput = -gamepad1.right_stick_x;

        // A tuşuna basılı tutulduğunda:
        if (gamepad1.a) {
            turnInput = Superstructure.updateTargetLockOrSweep(robotPose.heading(), follower.velocity().omega,
                    Math.hypot(follower.velocity().vx, follower.velocity().vy));
        } else {
            // Tuş bırakıldığında bir sonraki basış için hedef açıyı ve sweep merkezini sıfırla
            Superstructure.stopAim();
        }

        // Sürüş vektörünü uygula
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                turnInput,
                robotPose.heading()
        );

        follower.manual(powers);

        if (gamepad1.startWasPressed()) {
            Pose cornerPose = new Pose(8.7, 8.7, Math.toRadians(90));
            follower.setPose(cornerPose);
            // Hedef tahmini eski heading'e göreydi, sıfırla
            Superstructure.stopAim();
        }

        Superstructure.write();

        // Telemetri
        telemetry.addData("Durum", gamepad1.a ? (Superstructure.hasAimTarget() ? "KİLİTLENDİ" : "RADAR TARAMASI (±30°)") : "MANUEL");
        telemetry.addData("Tag Görünüyor mu", Superstructure.vision.tv);
        telemetry.addData("tx", Superstructure.vision.tx);
        telemetry.addData("Tag Mesafesi (cm)", "%.1f  (0 = tag yok ya da Limelight Full 3D kapalı)", Superstructure.getTagDistanceCm());
        telemetry.addData("Hizalandı mı", Superstructure.isAligned(follower.velocity().omega));
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.update();
    }
}