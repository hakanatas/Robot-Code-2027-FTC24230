package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * ÖRNEK OTONOM: 60 cm ileri git, 3 saniye atış yap, 90 derece sağa dön, 30 cm ileri git, 1 saniye bekle,
 * 1300 hızla 2 saniye atış yap, başlangıç yönüne dön ve başlangıç noktasına geri git.
 * Kendi otonomun için bu dosyayı kopyala, sınıfın ve @Autonomous'un adını değiştir, otonom()'u doldur.
 */
@Autonomous(name = "Ornek Otonom", group = "BioBuzz")
public class OrnekOtonom extends BioBuzzOtonom {

    @Override
    public void otonom() {
        ileriGit(60);
        atisYap(3);          // 3 saniye, varsayılan hız (1220)
        sagaDon(90);
        ileriGit(30);
        bekle(1);
        atisYap(2, 1300);    // 2 saniye, 1300 hızla
        aciyaDon(0);
        noktayaGit(0, 0, 0);
    }
}
