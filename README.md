# BerryBot IR Commands

BerryBot için MicroPython tabanlı kontrol yazılımı.

Bu repo, BerryBot'un IR kumanda ile manuel ve otonom modlarda kontrol edilmesini sağlar.

## İçerik

- `main.py`: Ana çalışma dosyası (IR/BLE modları, Raider/Follow/Guard/Sumo davranışları)
- `berrybot.py`: Donanım sürücüleri (motor, IR, LED matrix, WS2812, sensörler)
- `CHANGELOG.md`: Sürüm geçmişi ve önemli değişiklikler

## Özellikler

- IR kumanda ile hareket komutları
- Modlar:
  - Raider Mode (`0`)
  - Follow Mode (`7`)
  - Guard Mode (`9`)
  - Sumo Mode (`8`)
- LED matrix ikonları ve RGB efektleri
- Motor hareket logları
- Watchdog ve hata durumunda güvenli toparlanma

## Kurulum

1. MicroPython destekli BerryBot kartını bilgisayara bağlayın.
2. Bu repodaki dosyaları cihaza yükleyin.
3. `main.py` dosyasını cihazda çalıştırın.

## IR Tuşları (özet)

- `0`: Raider Mode ON/OFF
- `7`: Follow Mode ON/OFF
- `8`: Sumo Mode ON/OFF
- `9`: Guard Mode ON/OFF
- Yön tuşları: manuel hareket

## Bootloader Referansı

BerryBot bootloader yükleme/yeniden yükleme adımları için resmi kaynak:

- https://github.com/Robotistan/BerryBot/tree/main/Bootloader

Not: Bootloader işlemi yapmadan önce cihaz modelinizin doğru olduğundan emin olun.

## Geliştirme Notu

Proje aktif olarak geliştirilmiştir. Son değişiklikler için `CHANGELOG.md` dosyasına bakın.
