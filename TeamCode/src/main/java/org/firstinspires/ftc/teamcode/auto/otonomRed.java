package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * KIRMIZI İTTİFAK OTONOMU: başladığı yerde 1300 hızla 2 saniye atış yap, sonra 60 cm ileri git.
 * Komutların listesi için BioBuzzOtonom.java'nın başındaki açıklamaya bak.
 */
@Autonomous(name = "redOtonom", group = "BioBuzz")
public class otonomRed extends BioBuzzOtonom {

    @Override
    public void otonom() {
        atisYap(3, 1260);
        sagaGit(100);
        
    }
}
