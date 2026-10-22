package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

import java.util.ArrayList;
import java.util.List;

/*
 * HEDEF = YAN YANA DİZİLİ TAG'LERİN ORTASI (ör. 38, 39, 40, 41 -> atış kutusu 39 ile 40'ın arasında, tag'lerin üstünde).
 * tx, ty ve mesafe tek bir tag'in değil, bu grubun ORTASININ değerleridir.
 *
 * Tag'ler eşit aralıklı ve ID sırasıyla dizili olduğu için tx, ID'ye göre bir doğru üzerinde değişir:
 *     tx(ID) ≈ a + b * ID
 * Görünen tag'lerden a ve b'yi buluyoruz (en küçük kareler) ve grubun ortasının (ör. ID 39.5) açısını hesaplıyoruz.
 * Böylece robot yana kayıp bazı tag'ler görüşten çıksa bile hedef hep kutunun ortası kalır.
 * Tek tag görünüyorsa son bilinen eğim (b) kullanılır.
 *
 * Hangi tag'lerin hedef olduğu Superstructure.TARGET_TAG_GROUPS'ta yazılıdır.
 */
public class Vision extends WSubsystem {
    public Limelight3A LL3;
    // tx: hedefin ROBOT MERKEZİNE göre açısı (kamera kayıklığı düzeltilmiş, hizalama bunu kullanır)
    // txCamera: Limelight'ın gördüğü ham açı
    public double tx;
    public double txCamera;
    public double ty;
    public boolean tv;
    public double tl;
    // Bu döngüde yeni bir görüntü geldi mi, ve o görüntü ne zaman çekildi (System.currentTimeMillis cinsinden)
    public boolean newFrame;
    public long captureTimeMs;
    private double lastFrameTimestamp = -1;
    // Hedef gruptaki görünen tag'ler (ör. "38+39+40"), görünen tag sayısı ve
    // kameradan grubun ortasına mesafe (0: tag yok ya da Limelight 3D veri vermiyor)
    public String tagIds = "-";
    // Limelight'ın gördüğü BÜTÜN tag'ler (hedef listesinde olmasa bile), ayar kontrolü için
    public String allTagIds = "-";
    public int tagCount = 0;
    public double distanceCm;
    LLResult results;

    // TARGET_TAG_GROUPS'tan okunan gruplar (ör. [[38,39,40,41], [...]])
    private String parsedGroupsText = null;
    private List<int[]> groups = new ArrayList<>();
    // Hizalarken kilitlenen grubun sırası (-1: kilit yok) ve son bilinen eğim (derece / ID)
    private int lockedGroup = -1;
    private int currentGroup = -1;
    private double lastSlope = Double.NaN;
    // Son bilinen duvar (tag dizisi) yönü, yatay düzlemde ID başına cm: sağa, ileri
    private double lastWallRight = Double.NaN, lastWallForward = Double.NaN;
    // Kutu ağzına hangi açıdan bakıyoruz (0 = tam karşıdan) ve 16 cm düzeltmesi uygulanabildi mi
    public double boxViewAngleDeg = 0.0;
    public boolean boxOffsetApplied = false;

    @Override
    public void init(HardwareMap hardwareMap) {
        LL3 = hardwareMap.get(Limelight3A.class, "limelight");
        LL3.start();
    }

    /** Şu an hedeflenen grubu kilitle: tuş bırakılana kadar başka gruba (ör. diğer kaleye) geçilmesin. */
    public void lockGroup() {
        lockedGroup = currentGroup;
    }

    /** Kilidi kaldır: bir sonraki seferde grup yeniden seçilsin. */
    public void unlockGroup() {
        lockedGroup = -1;
        lastSlope = Double.NaN;
        lastWallRight = Double.NaN;
        lastWallForward = Double.NaN;
    }

    @Override
    public void periodic() {}

    @Override
    public void read() {
        parseGroups(Superstructure.TARGET_TAG_GROUPS);
        this.results = LL3.getLatestResult();

        // Hangi grubu hedefleyeceğiz: kilitliyse o, değilse en çok tag'i görünen grup
        StringBuilder all = new StringBuilder();
        if (results != null && results.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : results.getFiducialResults()) {
                if (all.length() > 0) all.append(',');
                all.append(fiducial.getFiducialId());
            }
        }
        this.allTagIds = all.length() > 0 ? all.toString() : "-";

        List<LLResultTypes.FiducialResult> visible = new ArrayList<>();
        int bestGroup = -1;
        if (results != null && results.isValid()) {
            for (int g = 0; g < groups.size(); g++) {
                if (lockedGroup >= 0 && g != lockedGroup) continue;
                List<LLResultTypes.FiducialResult> inGroup = new ArrayList<>();
                for (LLResultTypes.FiducialResult fiducial : results.getFiducialResults()) {
                    if (contains(groups.get(g), fiducial.getFiducialId())) {
                        inGroup.add(fiducial);
                    }
                }
                if (inGroup.size() > visible.size()) {
                    visible = inGroup;
                    bestGroup = g;
                }
            }
        }

        if (visible.isEmpty()) {
            // Hedef tag'lerden hiçbiri görünmüyor (başka tag'ler görünse bile onlara hizalanmıyoruz)
            this.tv = false;
            this.tx = 0.0;
            this.txCamera = 0.0;
            this.ty = 0.0;
            this.newFrame = false;
            this.tagIds = "-";
            this.tagCount = 0;
            this.distanceCm = 0.0;
            return;
        }
        currentGroup = bestGroup;

        // Görünen tag'lerin ID ve tx ortalamaları, ty ortalaması, mesafe ortalaması
        int n = visible.size();
        double meanId = 0, meanTx = 0, meanTy = 0, distanceSum = 0;
        int distanceCount = 0;
        StringBuilder ids = new StringBuilder();
        for (LLResultTypes.FiducialResult tag : visible) {
            meanId += (double) tag.getFiducialId() / n;
            meanTx += tag.getTargetXDegrees() / n;
            meanTy += tag.getTargetYDegrees() / n;
            // Kameradan tag merkezine düz çizgi mesafe (kameranın eğik takılmasından etkilenmez).
            // Limelight 3D veri hesaplamıyorsa konum (0, 0, 0) gelir, o tag atlanır.
            Position p = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.CM);
            double d = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (d > 0) {
                distanceSum += d;
                distanceCount++;
            }
            if (ids.length() > 0) ids.append('+');
            ids.append(tag.getFiducialId());
        }

        // Eğim b = ID başına kaç derece (en küçük kareler). En az 2 farklı ID görünüyorsa hesaplanabilir.
        double covariance = 0, variance = 0;
        for (LLResultTypes.FiducialResult tag : visible) {
            double dId = tag.getFiducialId() - meanId;
            covariance += dId * (tag.getTargetXDegrees() - meanTx);
            variance += dId * dId;
        }
        if (variance > 0) {
            lastSlope = covariance / variance;
        }

        // Grubun ortası (ör. 38..41 için ID 39.5) için tx
        double centerId = average(groups.get(bestGroup));
        double slope = Double.isNaN(lastSlope) ? 0.0 : lastSlope;
        this.txCamera = meanTx + slope * (centerId - meanId);

        this.tv = true;
        this.ty = meanTy;
        this.tl = results.getTargetingLatency() / 100.0;
        this.tagIds = ids.toString();
        this.tagCount = n;
        this.distanceCm = distanceCount > 0 ? distanceSum / distanceCount : 0.0;
        if (distanceCount == n) {
            // Bütün tag'lerin 3D konumu var: kutu ağzının (tag'lerden öne çıkık) gerçek yerine nişan al
            this.tx = boxMouthTx(visible, centerId);
        } else {
            this.tx = robotCenteredTx(txCamera, distanceCm);
            this.boxOffsetApplied = false;
        }

        // Limelight aynı görüntünün sonucunu birkaç kez gönderebilir; zaman damgası değiştiyse yeni görüntüdür
        this.newFrame = results.getTimestamp() != lastFrameTimestamp || results.getTimestamp() == 0;
        lastFrameTimestamp = results.getTimestamp();
        // Control Hub'a geliş anından kamera + işleme gecikmesini çıkarınca görüntünün çekildiği an
        this.captureTimeMs = results.getControlHubTimeStamp()
                - Math.round(results.getCaptureLatency() + results.getTargetingLatency());
    }

    // Kamera robotun merkezinde olmadığı için hedefin robot merkezine göre açısını hesapla.
    // Kameraya göre hedef: d uzaklıkta, txCam açısında. Robot merkezine göre hedef = kameranın yeri + bu vektör.
    // Örnek: kamera 9 cm solda, hedef kameranın tam karşısında 1 m'de -> hedef robot merkezinin ~4.4° solunda.
    private static double robotCenteredTx(double txCam, double distanceCm) {
        double d = distanceCm > 0 ? distanceCm : Superstructure.DEFAULT_TARGET_DISTANCE_CM;
        double leftAngle = Math.toRadians(-txCam); // tx pozitifse hedef sağda, sola doğru açı negatif
        double forward = Superstructure.CAMERA_FORWARD_OFFSET_CM + d * Math.cos(leftAngle);
        double left = Superstructure.CAMERA_LEFT_OFFSET_CM + d * Math.sin(leftAngle);
        return -Math.toDegrees(Math.atan2(left, forward));
    }

    /*
     * Kutunun ağzı tag'lerin ortasından BOX_MOUTH_OFFSET_CM kadar robota doğru öne çıkık.
     * Robot tam karşıdan bakarken bu fark nişanı değiştirmez, ama yandan bakarken ağız tag'lerin ortasıyla
     * aynı çizgide görünmez. Bu yüzden:
     * 1) Her tag'in 3D konumunu yatay düzleme çevir (kamera yukarı eğik takılı: CAMERA_PITCH_DEG).
     * 2) Tag'lerin dizildiği çizgiden duvarın yönünü ve grubun ortasını bul.
     * 3) Ortadan, duvara dik olarak robota doğru 16 cm git: kutunun ağzı.
     * 4) Bu noktanın robot merkezine göre açısını hesapla (kameranın kayıklığı da dahil).
     * Limelight kamera ekseni: x sağa, y aşağı, z kameranın baktığı yöne.
     */
    private double boxMouthTx(List<LLResultTypes.FiducialResult> visible, double centerId) {
        double pitch = Math.toRadians(Superstructure.CAMERA_PITCH_DEG);
        double scale = Superstructure.TAG_DISTANCE_SCALE;
        int n = visible.size();
        double[] ids = new double[n], right = new double[n], forward = new double[n];
        for (int i = 0; i < n; i++) {
            Position p = visible.get(i).getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.CM);
            ids[i] = visible.get(i).getFiducialId();
            right[i] = p.x * scale;
            forward[i] = (p.z * Math.cos(pitch) + p.y * Math.sin(pitch)) * scale; // yatay ileri mesafe
        }

        // Konum(ID) ≈ orta + yön * (ID - ortaID): sağa ve ileri için ayrı ayrı en küçük kareler
        double meanId = 0, meanRight = 0, meanForward = 0;
        for (int i = 0; i < n; i++) {
            meanId += ids[i] / n;
            meanRight += right[i] / n;
            meanForward += forward[i] / n;
        }
        double variance = 0, covRight = 0, covForward = 0;
        for (int i = 0; i < n; i++) {
            double dId = ids[i] - meanId;
            variance += dId * dId;
            covRight += dId * (right[i] - meanRight);
            covForward += dId * (forward[i] - meanForward);
        }
        if (variance > 0) {
            lastWallRight = covRight / variance;
            lastWallForward = covForward / variance;
        }

        double aimRight, aimForward;
        if (Double.isNaN(lastWallRight)) {
            // Tek tag görüldü, duvarın yönü henüz bilinmiyor: düzeltmesiz tag'e nişan al
            aimRight = meanRight;
            aimForward = meanForward;
            boxOffsetApplied = false;
            boxViewAngleDeg = 0.0;
        } else {
            double centerRight = meanRight + lastWallRight * (centerId - meanId);
            double centerForward = meanForward + lastWallForward * (centerId - meanId);
            // Duvara dik yön, robota doğru bakan taraf
            double length = Math.hypot(lastWallRight, lastWallForward);
            double normalRight = -lastWallForward / length;
            double normalForward = lastWallRight / length;
            if (normalRight * centerRight + normalForward * centerForward > 0) {
                normalRight = -normalRight;
                normalForward = -normalForward;
            }
            aimRight = centerRight + Superstructure.BOX_MOUTH_OFFSET_CM * normalRight;
            aimForward = centerForward + Superstructure.BOX_MOUTH_OFFSET_CM * normalForward;
            boxOffsetApplied = true;
            // 0 = kutuya tam karşıdan bakıyoruz; işaret hangi taraftan baktığımızı gösterir
            boxViewAngleDeg = Math.toDegrees(Math.atan2(normalRight, -normalForward));
        }

        // Robot merkezine göre: kamera robotun CAMERA_LEFT_OFFSET_CM solunda, CAMERA_FORWARD_OFFSET_CM önünde
        double robotForward = Superstructure.CAMERA_FORWARD_OFFSET_CM + aimForward;
        double robotLeft = Superstructure.CAMERA_LEFT_OFFSET_CM - aimRight;
        return -Math.toDegrees(Math.atan2(robotLeft, robotForward));
    }

    // "38,39,40,41; 42,43,44,45" -> [[38,39,40,41], [42,43,44,45]]. Sadece metin değişince yeniden okur.
    private void parseGroups(String text) {
        if (text.equals(parsedGroupsText)) return;
        parsedGroupsText = text;
        List<int[]> parsed = new ArrayList<>();
        for (String groupText : text.split(";")) {
            List<Integer> idList = new ArrayList<>();
            for (String idText : groupText.split(",")) {
                try {
                    idList.add(Integer.parseInt(idText.trim()));
                } catch (NumberFormatException ignored) {
                    // boş ya da hatalı yazılmış ID'yi atla
                }
            }
            if (!idList.isEmpty()) {
                int[] group = new int[idList.size()];
                for (int i = 0; i < group.length; i++) group[i] = idList.get(i);
                parsed.add(group);
            }
        }
        groups = parsed;
        lockedGroup = -1;
    }

    private static boolean contains(int[] group, int id) {
        for (int g : group) if (g == id) return true;
        return false;
    }

    private static double average(int[] group) {
        double sum = 0;
        for (int g : group) sum += g;
        return sum / group.length;
    }

    @Override
    public void write() {}

    @Override
    public void reset() {
        unlockGroup();
    }
}
