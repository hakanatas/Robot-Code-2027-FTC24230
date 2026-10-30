package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * ÖRNEK OTONOM: 60 cm ileri git, 90 derece sağa dön, 30 cm ileri git, 1 saniye bekle,
 * başlangıç yönüne dön ve başlangıç noktasına geri git.
 * Kendi otonomun için bu dosyayı kopyala, sınıfın ve @Autonomous'un adını değiştir, otonom()'u doldur.
 */
@Autonomous(name = "redOtonom", group = "BioBuzz")
public class otonomRed extends BioBuzzOtonom {

    @Override
    public void otonom() {
        atisYap(2, 1300);
        ileriGit(60);

    }
}
