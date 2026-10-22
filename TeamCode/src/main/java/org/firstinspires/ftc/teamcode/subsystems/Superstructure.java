package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * HİZALAMA NASIL ÇALIŞIYOR:
 * 1) Limelight'tan her yeni görüntü geldiğinde, o görüntünün ÇEKİLDİĞİ ANDAKİ robot açısını kayıttan buluyoruz.
 *    hedefAçı = o andaki robotAçısı - tx. Böylece Limelight gecikmesi robot dönerken bile hata yaratmıyor.
 * 2) Robot sürülürken (sağa sola giderken) hedef robota göre sürekli kayar. Ardışık ölçümlerden hedefin
 *    ne hızla kaydığını tahmin ediyoruz (alpha-beta filtresi) ve robotu baştan o hızla döndürüyoruz.
 * 3) Robotu bu hareketli hedef açıya Pinpoint heading'i ile PD + hız ileri beslemesi ile çeviriyoruz.
 *
 * ESKİSİNDE tx ile direkt PID yapıyorduk: gecikme yüzünden kazanç düşük kalıyordu (robot yavaş/güçsüzdü)
 * ve robot hareket ederken hedefi takip edemiyordu.
 *
 * Sabitler Panels'ten canlı ayarlanabilir: 192.168.43.1:8001 > Configurables > Superstructure
 */
@Configurable
public class Superstructure {
    @IgnoreConfigurable
    public static Vision vision = new Vision();

    // güç = KV * hedefHızı + KP * hata + KD * (hedefHızı - robotHızı)   (açılar radyan, hızlar radyan/s)
    public static double HEADING_KP = 3.0;  // 1 derece hata ≈ 0.052 güç (eski tx PID: 0.02)
    public static double HEADING_KD = 0.2;  // salınıyorsa artır, hedefe çok yavaş yaklaşıyorsa azalt
    // Hedef kayarken robotu o hızda döndürmek için gereken güç (1 rad/s ≈ 57 derece/s başına).
    // Robot hareket halinde hedefin GERİSİNDE kalıyorsa artır, ÖNÜNE geçiyorsa azalt.
    public static double HEADING_KV = 0.15;

    // Sürtünmeyi yenmek için minimum güç. SADECE robot dururken ve hata yönünde eklenir.
    public static double HEADING_KS = 0.04;
    // ROBOT MEVLANA MOD AÇMASIN DİYE ÜST SINIR
    public static double MAX_TURN_POWER = 0.6;
    // Hedefe bu kadar yakınsa (ve hedef de robot da duruyorsa) motoru durdur
    public static double HOLD_DEADBAND_DEG = 1.0;
    // Durduktan sonra hata bunu geçmedikçe tekrar hareket etme (titremeyi önleyen histerezis).
    // ALIGNMENT_TOLERANCE_DEG'den küçük olmalı ki duran robot hep "hizalı" sayılsın.
    public static double HOLD_RESTART_DEG = 1.5;

    // tx bu toleransın içindeyse VE robot hedefle aynı hızda dönüyorsa hizalandı sayılır, atış ancak o zaman başlar
    public static double ALIGNMENT_TOLERANCE_DEG = 2.0;
    public static double ALIGNED_MAX_TURN_SPEED_DEG = 15.0; // derece/saniye, robot hızı ile hedef hızı farkı

    // Bundan yavaş dönen robot/hedef "duruyor" sayılır (derece/saniye)
    public static double STOPPED_TURN_SPEED_DEG = 10.0;
    // Robot bundan yavaş sürülüyorsa (inç/saniye) hedef robota göre kayamaz, hedef hızı 0 kabul edilir.
    // Böylece atış titreşimi ya da tx gürültüsü hayali bir hedef hızına dönüşüp robotu sağa sola savurmaz.
    public static double STOPPED_DRIVE_SPEED = 4.0;

    // Hedef tag grupları: robot grubun ORTASINA hizalanır (atış kutusu tag'lerin ortasının üstünde).
    // Tag'ler soldan sağa ya da sağdan sola ID sırasıyla, eşit aralıklı dizili olmalı.
    // Birden fazla grup (ör. iki ittifakın kalesi) ';' ile ayrılır: "38,39,40,41; 42,43,44,45"
    // Burada olmayan tag'ler görünse bile onlara hizalanılmaz.
    public static String TARGET_TAG_GROUPS = "38,39,40,41";

    // Limelight robotun merkezinde değil. Robotun MERKEZİNİ hedefe çevirmek için kameranın yeri (cm):
    // sola doğru + (sağdaysa eksi yaz), ileri doğru + (Limelight ayarlarındaki "LL Forward" 0.16 m).
    public static double CAMERA_LEFT_OFFSET_CM = 9.0;
    public static double CAMERA_FORWARD_OFFSET_CM = 16.0;
    // Tag mesafesi okunamazsa (Full 3D kapalı) düzeltme bu mesafe varsayılarak yapılır
    public static double DEFAULT_TARGET_DISTANCE_CM = 150.0;
    // Limelight'ın yukarı bakış açısı (Limelight ayarlarındaki "LL Pitch")
    public static double CAMERA_PITCH_DEG = 24.624;
    // Kutunun ağzı tag'lerin ortasından robota doğru bu kadar öne çıkık (cm)
    public static double BOX_MOUTH_OFFSET_CM = 16.0;

    // Hedef açı filtresi: ALPHA yeni ölçüme ne kadar güvenileceği (0-1), BETA hedef hızının ne kadar çabuk güncelleneceği
    public static double TARGET_FILTER_ALPHA = 0.4;
    public static double TARGET_FILTER_BETA = 0.1;
    // Robot bundan hızlı dönerken ölçüm zamanlamasındaki küçük hatalar büyür, hedef hızını güncelleme
    public static double RATE_UPDATE_MAX_TURN_SPEED_DEG = 60.0;
    public static double MAX_TARGET_RATE_DEG = 90.0; // derece/saniye
    // Tag bu kadar süre görünmezse hedefin kaymaya devam ettiğini varsayma, son açıda dur
    public static double TARGET_TIMEOUT = 0.3; // saniye

    // Tag mesafesi düzeltme çarpanı = şeritmetreyle ölçülen mesafe / telemetry'deki mesafe.
    // Limelight'taki alan haritası eski sezondan (tag'ler 165.1 mm, bizimkiler 82.55 mm). Limelight mesafeyi
    // haritadaki boyutla hesaplarsa mesafe 2 kat çıkar, o zaman bunu 0.5 yap.
    // Şeritmetre ölçümü: telemetry 144 cm, gerçek 147 cm -> 147 / 144 = 1.021
    public static double TAG_DISTANCE_SCALE = 1.021;

    // SAĞA SOLA TARAMA YAPMAK İÇİN KULLANDIM
    public static double SWEEP_AMPLITUDE_DEG = 30.0; // ±30 Derece
    public static double SWEEP_POWER = 0.15;

    private static boolean isSweeping = false;
    private static double centerHeading = 0.0;
    private static int sweepDirection = 1;

    // Hedefin son ölçüm anındaki açısı ve kayma hızı
    private static boolean hasTargetHeading = false;
    private static double targetHeading = 0.0;
    private static double targetRate = 0.0;
    private static long targetTimeMs = 0;
    private static boolean isHolding = false;
    // Atış başladı: hedef açı donduruldu, Limelight ile yeniden hizalanılmıyor
    private static boolean aimFrozen = false;

    // Son ~0.5-1 saniyenin robot açıları: görüntünün çekildiği andaki açıyı bulmak için
    private static final int HISTORY_SIZE = 50;
    private static final long[] historyTimes = new long[HISTORY_SIZE];
    private static final double[] historyHeadings = new double[HISTORY_SIZE];
    private static int historyCount = 0;
    private static int historyNext = 0;

    public static void init(HardwareMap hardwareMap) {
        vision.init(hardwareMap);
    }

    public static void periodic() {
        vision.periodic();
    }

    public static void read() {
        vision.read();
    }

    public static void write() {
        vision.write();
    }

    public static void reset() {
        vision.reset();
        stopAim();
    }

    /**
     * Tuş bırakıldığında (ve heading sıfırlandığında) çağır: bir sonraki basışta her şey sıfırdan başlasın.
     */
    public static void stopAim() {
        isSweeping = false;
        hasTargetHeading = false;
        targetRate = 0.0;
        isHolding = false;
        aimFrozen = false;
        historyCount = 0;
        vision.unlockGroup();
    }

    /**
     * Atış başladığında çağır: robot o anki açısında kalsın, tuş bırakılana kadar Limelight'a göre
     * yeniden hizalanmasın (atış titreşimi tx'i oynatınca robot tekrar dönmeye başlıyordu).
     * Robot atışın tepkisiyle 1.5 dereceden fazla dönerse Pinpoint ile aynı açıya geri döner.
     */
    public static void freezeAim(double currentHeading) {
        if (aimFrozen) return;
        aimFrozen = true;
        hasTargetHeading = true;
        targetHeading = currentHeading;
        targetRate = 0.0;
        targetTimeMs = System.currentTimeMillis();
        isHolding = true;
    }

    public static boolean isAimFrozen() {
        return aimFrozen;
    }

    /**
     * Tag görüldüyse hedefe kilitlenir ve robot hareket ederken de takip eder (tag kaybolursa son açıda kalır),
     * hiç görülmediyse ±30 derece radar taraması yapar.
     * @param currentHeading Robotun anlık açısı (radyan)
     * @param angularVelocity Robotun dönüş hızı (radyan/saniye, follower.velocity().omega)
     * @param driveSpeed Robotun ileri/yana sürülme hızı (inç/saniye)
     * @return Motor dönüş gücü (Pedro: + saat yönünün tersi)
     */
    public static double updateTargetLockOrSweep(double currentHeading, double angularVelocity, double driveSpeed) {
        long now = System.currentTimeMillis();
        recordHeading(now, currentHeading);

        // Atış sırasında: sadece dondurulan açıyı koru
        if (aimFrozen) {
            return turnToTarget(now, currentHeading, angularVelocity);
        }

        // DURUM 1: Tag Görünüyor -> her yeni görüntüde hedef tahminini güncelle
        if (vision.tv) {
            isSweeping = false;
            if (vision.newFrame) {
                updateTargetEstimate(vision.captureTimeMs, currentHeading, angularVelocity, driveSpeed);
                // Bu basış boyunca hep aynı tag grubunun ortasını takip et (başka tag'e atlamasın)
                vision.lockGroup();
            }
        }

        // Hedef biliniyorsa Pinpoint ile hedefe dön ve takip et
        if (hasTargetHeading) {
            return turnToTarget(now, currentHeading, angularVelocity);
        }

        // DURUM 2: Tag Hiç Görülmedi -> ±30 Taraması
        if (!isSweeping) {
            centerHeading = currentHeading;
            isSweeping = true;
            sweepDirection = 1;
        }

        double headingError = normalizeAngle(currentHeading - centerHeading);

        if (headingError >= Math.toRadians(SWEEP_AMPLITUDE_DEG)) {
            sweepDirection = -1;
        } else if (headingError <= -Math.toRadians(SWEEP_AMPLITUDE_DEG)) {
            sweepDirection = 1;
        }

        return SWEEP_POWER * sweepDirection;
    }

    private static void updateTargetEstimate(long captureTimeMs, double currentHeading,
                                             double angularVelocity, double driveSpeed) {
        // Görüntü çekildiğinde robot hangi açıdaydı? Hedefin o andaki açısı = o açı - tx
        // (tx pozitifse hedef sağda: saat yönünde dönmek gerek, yani heading azalmalı)
        double measured = normalizeAngle(headingAt(captureTimeMs, currentHeading) - Math.toRadians(vision.tx));

        double dt = (captureTimeMs - targetTimeMs) / 1000.0;
        if (!hasTargetHeading || dt > TARGET_TIMEOUT) {
            // İlk görüş ya da tag uzun süre kayıptı: sıfırdan başla
            targetHeading = measured;
            targetRate = 0.0;
            targetTimeMs = captureTimeMs;
            hasTargetHeading = true;
            return;
        }
        if (dt <= 0) {
            return;
        }

        // Alpha-beta filtresi: hedefin şimdi nerede olması gerektiğini tahmin et, ölçümle farkının bir kısmı kadar düzelt
        double predicted = targetHeading + targetRate * dt;
        double residual = normalizeAngle(measured - predicted);
        targetHeading = normalizeAngle(predicted + TARGET_FILTER_ALPHA * residual);

        if (Math.abs(driveSpeed) < STOPPED_DRIVE_SPEED) {
            // Robot sürülmüyor: hedef kayamaz
            targetRate = 0.0;
        } else if (Math.abs(angularVelocity) < Math.toRadians(RATE_UPDATE_MAX_TURN_SPEED_DEG)) {
            // Çok sık gelen görüntülerde dt küçülüp gürültüyü büyütmesin diye en az 30 ms say
            targetRate += TARGET_FILTER_BETA * residual / Math.max(dt, 0.03);
            double maxRate = Math.toRadians(MAX_TARGET_RATE_DEG);
            targetRate = Math.max(-maxRate, Math.min(maxRate, targetRate));
        }
        targetTimeMs = captureTimeMs;
    }

    private static double turnToTarget(long now, double currentHeading, double angularVelocity) {
        // Son ölçümden beri hedef ne kadar kaydı (tag kaybolursa en fazla TARGET_TIMEOUT kadar ileri tahmin et)
        double age = Math.max(0.0, (now - targetTimeMs) / 1000.0);
        double rate = currentTargetRate(now);
        double target = normalizeAngle(targetHeading + targetRate * Math.min(age, TARGET_TIMEOUT));

        double error = normalizeAngle(target - currentHeading);
        double stoppedSpeed = Math.toRadians(STOPPED_TURN_SPEED_DEG);
        boolean robotStopped = Math.abs(angularVelocity) < stoppedSpeed;
        boolean targetStopped = Math.abs(rate) < stoppedSpeed;

        // Hedefte durduysa, hata HOLD_RESTART_DEG'i geçmedikçe kıpırdama. Yoksa küçük düzeltmelerde
        // minimum güç (KS) robotu hedefin öbür tarafına atıyor ve robot olduğu yerde titriyordu.
        if (isHolding) {
            if (Math.abs(error) <= Math.toRadians(HOLD_RESTART_DEG) && targetStopped) {
                return 0.0;
            }
            isHolding = false;
        }

        if (Math.abs(error) <= Math.toRadians(HOLD_DEADBAND_DEG) && robotStopped && targetStopped) {
            isHolding = true;
            return 0.0;
        }

        double turnPower = HEADING_KV * rate + HEADING_KP * error + HEADING_KD * (rate - angularVelocity);

        // Robot duruyorsa sürtünmeyi yenmesi için hata yönünde küçük bir itiş ekle
        if (robotStopped) {
            turnPower += Math.signum(error) * HEADING_KS;
        }

        return Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, turnPower));
    }

    // Tag yakın zamanda görüldüyse hedefin kayma hızı, yoksa 0 (radyan/saniye)
    private static double currentTargetRate(long now) {
        if (!hasTargetHeading || (now - targetTimeMs) / 1000.0 > TARGET_TIMEOUT) {
            return 0.0;
        }
        return targetRate;
    }

    private static void recordHeading(long timeMs, double heading) {
        historyTimes[historyNext] = timeMs;
        historyHeadings[historyNext] = heading;
        historyNext = (historyNext + 1) % HISTORY_SIZE;
        historyCount = Math.min(historyCount + 1, HISTORY_SIZE);
    }

    // Kayıttaki iki ölçüm arasında doğrusal ara değer bularak robotun timeMs anındaki açısı
    private static double headingAt(long timeMs, double currentHeading) {
        if (historyCount == 0) {
            return currentHeading;
        }
        int newest = (historyNext - 1 + HISTORY_SIZE) % HISTORY_SIZE;
        if (timeMs >= historyTimes[newest]) {
            return historyHeadings[newest];
        }
        for (int i = 1; i < historyCount; i++) {
            int later = (newest - i + 1 + HISTORY_SIZE) % HISTORY_SIZE;
            int earlier = (newest - i + HISTORY_SIZE) % HISTORY_SIZE;
            if (timeMs >= historyTimes[earlier]) {
                long span = historyTimes[later] - historyTimes[earlier];
                double t = span > 0 ? (double) (timeMs - historyTimes[earlier]) / span : 1.0;
                return historyHeadings[earlier]
                        + normalizeAngle(historyHeadings[later] - historyHeadings[earlier]) * t;
            }
        }
        // Kayıttan daha eski bir an: elimizdeki en eski açıyı kullan
        return historyHeadings[(newest - historyCount + 1 + HISTORY_SIZE) % HISTORY_SIZE];
    }

    /**
     * Kameradan hizalanan tag'e mesafe (cm), TAG_DISTANCE_SCALE ile düzeltilmiş. 0: tag yok ya da Full 3D kapalı.
     */
    public static double getTagDistanceCm() {
        return vision.distanceCm * TAG_DISTANCE_SCALE;
    }

    /**
     * Hedef açı bulundu mu (tag bu basışta en az bir kez görüldü mü)
     */
    public static boolean hasAimTarget() {
        return hasTargetHeading;
    }

    /**
     * Hedefin robota göre kayma hızı (radyan/saniye). Robot sürülürken sıfırdan farklıdır.
     */
    public static double getTargetRate() {
        return currentTargetRate(System.currentTimeMillis());
    }

    /**
     * Gerçek tx verisine dayalı kesin hizalanma kontrolü: tag görünüyor, tx tolerans içinde
     * ve robot hedefle aynı hızda dönüyor (duran hedefte: robot neredeyse durmuş)
     * @param angularVelocity Robotun dönüş hızı (radyan/saniye)
     */
    public static boolean isAligned(double angularVelocity) {
        return vision.tv
                && Math.abs(vision.tx) <= ALIGNMENT_TOLERANCE_DEG
                && Math.abs(angularVelocity - getTargetRate()) <= Math.toRadians(ALIGNED_MAX_TURN_SPEED_DEG);
    }

    // BU FONKSİYON İSE ROBOT AÇILARINI NORMALİZE ETMEK İÇİN -180,+180 OLARAK ALGILAMAK İÇİN YAKIN TARAFTAN DÖNEMK İÇİN KULLANIYORUM
    private static double normalizeAngle(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}
