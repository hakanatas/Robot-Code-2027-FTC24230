package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.OpModeStorage;

/*
 * ÖĞRENCİLER İÇİN BASİT OTONOM KOMUTLARI
 *
 * Kendi otonomunu yazmak için bu sınıftan türet ve otonom() fonksiyonunu doldur:
 *
 *     @Autonomous(name = "Benim Otonomum")
 *     public class BenimOtonomum extends BioBuzzOtonom {
 *         @Override
 *         public void otonom() {
 *             ileriGit(60);    // 60 cm ileri
 *             sagaDon(90);     // yerinde 90 derece sağa dön
 *             ileriGit(30);
 *             bekle(1);        // 1 saniye bekle
 *         }
 *     }
 *
 * Komutlar:
 *   ileriGit(cm)  geriGit(cm)  sagaGit(cm)  solaGit(cm)   -> robotun baktığı yöne göre, dönmeden
 *   sagaDon(derece)  solaDon(derece)                        -> yerinde dön
 *   aciyaDon(derece)                                        -> başlangıç yönüne göre o açıya dön (0 = ilk baktığı yön)
 *   noktayaGit(ileriCm, solaCm, derece)                     -> başlangıç noktasına göre bir noktaya git
 *   bekle(saniye)
 *   atisYap(saniye)        -> o kadar saniye R1'e basılı tutmuş gibi atış yap (hız: VARSAYILAN_ATIS_HIZI)
 *   atisYap(saniye, hiz)   -> aynısı, launcher hızını sen ver (ticks/s, teleop'taki "Launcher target")
 *
 * Her komut bitene kadar bekler, sonra bir sonraki satıra geçilir.
 * Robot sürüş ve dönüşü Pedro Pathing ile yapar (Pinpoint odometri + pedro/Constants.java'daki ayarlar).
 * Komutlar hep bir önceki komutun HEDEFİNDEN devam eder; böylece küçük hatalar birikmez.
 */
public abstract class BioBuzzOtonom extends LinearOpMode {

    // Ayarlar
    public static double DONUS_TOLERANSI_DERECE = 2.0;  // dönüş bu kadar yakınsa bitti sayılır
    public static double KONUM_TOLERANSI_CM = 2.0;      // gidiş bu kadar yakınsa bitti sayılır
    public static double ORTALAMA_HIZ_CM_SN = 60.0;     // zaman aşımı hesabı için
    public static double EK_SURE_SN = 2.0;              // her komuta eklenen güvenlik süresi
    public static int VARSAYILAN_ATIS_HIZI = 1220;      // ticks/s, teleop'taki varsayılan hız
    public static int ATIS_HIZ_PAYI = 50;               // launcher hedef hıza bu kadar yaklaşınca top beslenir

    protected Follower follower;
    protected DcMotorEx launcher;
    protected DcMotor intake;
    protected CRServo windmillServo, leftIntakeServo, rightIntakeServo;

    // Bir önceki komutun bittiği hedef konum (Pedro birimi: inç, radyan; başlangıç = 0,0,0)
    private Pose hedef = Pose.zero();

    /** Öğrencilerin dolduracağı kısım. */
    public abstract void otonom();

    /** Mekanizmaları (intake, launcher...) hazırlamak için; istersen override et. */
    public void hazirla() {}

    @Override
    public final void runOpMode() {
        follower = Constants.create(hardwareMap);
        mekanizmalariKur();
        hazirla();

        telemetry.addData("Durum", "Hazır, START'a bas");
        telemetry.update();
        waitForStart();

        // Robotun START anındaki yeri başlangıç noktası, baktığı yön 0 derece
        follower.setPose(Pose.zero());
        follower.update();
        hedef = Pose.zero();

        if (opModeIsActive()) {
            otonom();
        }

        follower.stop();
        atisMekanizmasiniDurdur();
        // Teleop'un robotun nerede kaldığını bilmesi için
        OpModeStorage.autonomousEndPose = follower.pose();
    }

    // ---------------- GİTME KOMUTLARI ----------------

    public void ileriGit(double cm) {
        robotaGoreGit(cm, 0);
    }

    public void geriGit(double cm) {
        robotaGoreGit(-cm, 0);
    }

    public void sagaGit(double cm) {
        robotaGoreGit(0, -cm);
    }

    public void solaGit(double cm) {
        robotaGoreGit(0, cm);
    }

    /** Başlangıç noktasına göre bir noktaya git (ileri +, sola +) ve verilen açıya dön. */
    public void noktayaGit(double ileriCm, double solaCm, double derece) {
        Pose yeniHedef = new Pose(cmToInch(ileriCm), cmToInch(solaCm), Math.toRadians(derece));
        git(yeniHedef);
    }

    // ---------------- DÖNME KOMUTLARI ----------------

    public void sagaDon(double derece) {
        don(hedef.heading() - Math.toRadians(derece));
    }

    public void solaDon(double derece) {
        don(hedef.heading() + Math.toRadians(derece));
    }

    /** Başlangıçta baktığı yöne göre bu açıya dön (sola +, sağa -). */
    public void aciyaDon(double derece) {
        don(Math.toRadians(derece));
    }

    // ---------------- ATIŞ ----------------

    /** saniye boyunca R1'e basılı tutmuş gibi atış yap, varsayılan hızla. */
    public void atisYap(double saniye) {
        atisYap(saniye, VARSAYILAN_ATIS_HIZI);
    }

    /**
     * saniye boyunca R1'e basılı tutmuş gibi atış yap: launcher hiz'a (ticks/s) çıkar,
     * hıza ulaşınca windmill ve intake topları besler. Süre launcher'ın hızlanma süresini de içerir.
     * Robot bu sırada yerinde durur (Limelight ile hizalanmaz). Süre bitince hepsi durur.
     */
    public void atisYap(double saniye, double hiz) {
        ElapsedTime zaman = new ElapsedTime();
        launcher.setVelocity(hiz);
        while (opModeIsActive() && zaman.seconds() < saniye) {
            follower.update();
            boolean hizda = launcher.getVelocity() > hiz - ATIS_HIZ_PAYI;
            // Teleop'taki R1 ile aynı: hızdayken windmill tam güç, intake yarım güç besler
            windmillServo.setPower(hizda ? 1.0 : 0.0);
            intakeGucu(hizda ? 0.5 : 0.0);
            telemetry.addData("Durum", "Atış yapıyor (%.1f / %.1f sn)", zaman.seconds(), saniye);
            telemetry.addData("Launcher hızı", "%.0f / %.0f %s", launcher.getVelocity(), hiz, hizda ? "(besleniyor)" : "(hızlanıyor)");
            telemetry.update();
        }
        atisMekanizmasiniDurdur();
    }

    // ---------------- BEKLEME ----------------

    /** Robot yerini korurken bekle. */
    public void bekle(double saniye) {
        ElapsedTime zaman = new ElapsedTime();
        while (opModeIsActive() && zaman.seconds() < saniye) {
            follower.update();
            durumuYaz("Bekliyor");
        }
    }

    // ---------------- İÇ İŞLEYİŞ (öğrencilerin değiştirmesi gerekmez) ----------------

    // Teleop'taki (BioBuzzFieldCentricTeleop) ile aynı donanım isimleri ve ayarları
    private void mekanizmalariKur() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        rightIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        atisMekanizmasiniDurdur();
    }

    private void intakeGucu(double guc) {
        intake.setPower(guc);
        leftIntakeServo.setPower(guc);
        rightIntakeServo.setPower(guc);
    }

    private void atisMekanizmasiniDurdur() {
        launcher.setVelocity(0);
        windmillServo.setPower(0);
        intakeGucu(0);
    }

    // Robotun hedef yönüne göre ileri/sola kaydırılmış yeni hedefe, yön değiştirmeden git
    private void robotaGoreGit(double ileriCm, double solaCm) {
        double h = hedef.heading();
        double ileri = cmToInch(ileriCm);
        double sola = cmToInch(solaCm);
        Pose yeniHedef = new Pose(
                hedef.x() + ileri * Math.cos(h) - sola * Math.sin(h),
                hedef.y() + ileri * Math.sin(h) + sola * Math.cos(h),
                h);
        git(yeniHedef);
    }

    // Şu anki konumdan yeniHedef'e düz çizgi yol oluştur ve takip et
    private void git(Pose yeniHedef) {
        if (!opModeIsActive()) return;
        Pose baslangic = follower.pose();
        double mesafeCm = inchToCm(baslangic.distance(yeniHedef));
        double zamanAsimi = EK_SURE_SN + mesafeCm / ORTALAMA_HIZ_CM_SN;

        if (mesafeCm > KONUM_TOLERANSI_CM) {
            follower.follow(line(baslangic, yeniHedef).linear(baslangic, yeniHedef));
        } else {
            follower.hold(yeniHedef);
        }
        hedef = yeniHedef;

        ElapsedTime zaman = new ElapsedTime();
        // Yol bitene kadar takip et (Pedro yol bitince hedefte durmaya geçer)
        while (opModeIsActive() && follower.following() && zaman.seconds() < zamanAsimi) {
            follower.update();
            durumuYaz("Gidiyor");
        }
        // Hedefe yerleşmesini bekle
        follower.hold(yeniHedef);
        bitmesiniBekle(zaman, zamanAsimi);
    }

    private void don(double hedefAci) {
        if (!opModeIsActive()) return;
        hedef = hedef.withHeading(hedefAci);
        follower.hold(hedef);
        ElapsedTime zaman = new ElapsedTime();
        bitmesiniBekle(zaman, EK_SURE_SN + 1.0);
    }

    // Konum ve açı toleransa girene kadar (ya da zaman aşımına kadar) hedefte tut
    private void bitmesiniBekle(ElapsedTime zaman, double zamanAsimi) {
        while (opModeIsActive() && zaman.seconds() < zamanAsimi) {
            follower.update();
            Pose p = follower.pose();
            double konumHatasiCm = inchToCm(p.distance(hedef));
            double aciHatasi = Math.toDegrees(Math.abs(aciFarki(hedef.heading(), p.heading())));
            durumuYaz("Yerleşiyor");
            if (konumHatasiCm < KONUM_TOLERANSI_CM && aciHatasi < DONUS_TOLERANSI_DERECE) {
                break;
            }
        }
    }

    private void durumuYaz(String durum) {
        Pose p = follower.pose();
        telemetry.addData("Durum", durum);
        telemetry.addData("Konum (cm)", "ileri %.1f  sola %.1f", inchToCm(p.x()), inchToCm(p.y()));
        telemetry.addData("Açı (derece)", "%.1f", Math.toDegrees(p.heading()));
        telemetry.addData("Hedef (cm)", "ileri %.1f  sola %.1f  açı %.1f",
                inchToCm(hedef.x()), inchToCm(hedef.y()), Math.toDegrees(hedef.heading()));
        telemetry.update();
    }

    private static double aciFarki(double a, double b) {
        double d = a - b;
        while (d > Math.PI) d -= 2 * Math.PI;
        while (d < -Math.PI) d += 2 * Math.PI;
        return d;
    }

    private static double cmToInch(double cm) {
        return cm / 2.54;
    }

    private static double inchToCm(double inch) {
        return inch * 2.54;
    }
}
