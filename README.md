# Kilya IMU Serial Test

Sadece IMU: Bluetooth ve Wi-Fi kapalı, CPU 80 MHz. Amaç, kartın radyo olmadan
(daha düşük akımla) uzun süre çalışıp çalışmadığını görmek; yani "1-2 dakika sonra
kesilme" sorununun besleme/LDO kaynaklı olup olmadığını ayırmak.

Sensör sürücüsü `Kilya_Base_Code`'dan kopyalandı (`components/lsm6dsox`); iki proje bağımsız.

```
idf.py set-target esp32c3
idf.py -p COMx flash monitor
```

## Çıktı
- Veri satırları (200 ms'de bir, periyot ortalaması, CSV):
  `t_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,pitch_deg,roll_deg,imu_temp_c`
- `#` ile başlayan satırlar: açılış bilgisi, **reset sebebi** ve 5 sn'de bir sağlık satırı
  (`up` çalışma süresi, IMU örnekleme hızı, I2C hataları, çip ve IMU sıcaklığı, boş bellek)

pitch/roll yalnızca ivmeölçerden, sensör eksenlerinde hesaplanır (kalibrasyon yok); sadece kontrol amaçlı.

## Yorumlama
- `up` süresi 1-2 dakikayı rahatça geçiyorsa: radyo akımı olmadan kart dayanıyor → LDO/besleme yükü sorunu.
- Yine 1-2 dakikada kesiliyorsa: radyodan bağımsız; `chip` sıcaklığına ve açılıştaki
  `reset reason` satırına bakın (BROWNOUT = besleme çöküyor).
