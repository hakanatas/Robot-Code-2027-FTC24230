package org.firstinspires.ftc.teamcode.lib;


import org.firstinspires.ftc.teamcode.lib.math.InterpolatingDouble;
import org.firstinspires.ftc.teamcode.lib.math.InterpolatingTreeMap;

public class Constants {
    public static class ShootingParams{
        public static double[][] kRPMValues = {
                {13.85,2800},
                {0.0,3000},
                {-7.5,3100}, // SONRADAN EKLENDİ DÜZELTME
                {-10.5,3450}, // SONRADAN EKLENDİ DÜZELTME
                {-12.55,3500},
                {-16.5,4100},
                {-17.0,4300}
        };
        public static double[][] kHoodValues = {
                {13.85,0.0},
                {0.0,10.0},
                {-12.55,20},
                {-16.5,32.0},
                {-17.0,32.0}
        };
        // Tag mesafesi (cm, TAG_DISTANCE_SCALE ile düzeltilmiş, telemetry'deki "Tag distance") ->
        // topun kutuya girdiği launcher hızı (ticks/s). Ara mesafeler iki nokta arasından hesaplanır,
        // tablonun dışındaki mesafelerde en yakın uçtaki hız kullanılır. Sahada yeni nokta buldukça ekle.
        public static double[][] kLauncherVelocityValues = {
                {131.7, 1220}, // 129 cm okundu (düzeltmeden önce), 1220 girdi
                {156.8, 1220}, // 153.6 cm okundu (düzeltmeden önce), 1220 girdi
        };
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kLauncherVelocityMap = new InterpolatingTreeMap<>();
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodMap = new InterpolatingTreeMap<>();
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kRPMMap = new InterpolatingTreeMap<>();
        static {
            for (double[] pair : kRPMValues) {
                kRPMMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
            }

            for (double[] pair : kLauncherVelocityValues) {
                kLauncherVelocityMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
            }

            for (double[] pair : kHoodValues) {
                kHoodMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
            }
        }
    }

}
