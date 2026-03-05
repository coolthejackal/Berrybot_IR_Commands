# BerryBot IR Commands

BerryBot icin MicroPython tabanli kontrol yazilimi.

Bu repo, BerryBot'un IR kumanda ile hareket ve otonom modlarda kontrol edilmesini saglar.

## Icerik

- `main.py`: Ana uygulama (IR/BLE, Movement ve Music mode, Raider/Follow/Guard/Sumo/Crazy)
- `berrybot.py`: Donanim suruculeri (motor, IR, LED matrix, WS2812, sensorler)
- `CHANGELOG.md`: Degisiklik kayitlari

## Kurulum

1. MicroPython destekli BerryBot kartini bilgisayara baglayin.
2. Bu repodaki dosyalari cihaza yukleyin.
3. `main.py` dosyasini calistirin.

## Modlar

- `Movement Mode` (varsayilan): Robot hareket ve otonom mod komutlarini alir.
- `Music Mode`: Robot hareket etmez, tuslara gore melodi calar.

Mod gecisi:

- `*` (`number_star`): `Music Mode ON`
- `#` (`number_sharp`): `Movement Mode ON`

## IR Tuslari

### Movement Mode

- `0`: Raider Mode ON/OFF
- `7`: Follow Mode ON/OFF
- `8`: Sumo Mode ON/OFF
- `9`: Guard Mode ON/OFF
- `4`: Crazy Mode ON/OFF
- `1`: Sola donus rutini
- `2`: Onde engel gorene kadar ileri git rutini
- `3`: Saga donus rutini
- `5`: Geri git (sensor kontrollu erken durma)
- `6`: Uzgun ikon + kisa ses
- Yon tuslari: Manuel hareket

### Music Mode

- Hareket komutlari calismaz (motor hareketi yoktur).
- Matrix'te muzik modu ikonu gorunur.
- Rakam tusuna basinca melodi boyunca ilgili rakam matrix'te kalir.

Atanan melodiler (her biri 3 tekrar):

- `1`: Happy Birthday To You
- `2`: Jingle Bells
- `3`: Clap Your Hands

## Muzik Sirasinda LED Davranisi

- Yan RGB LED'ler muzige senkron renkli blink yapar.
- Sarkiya ozel renk temalari kullanilir:
  - `1` Happy Birthday: sicak/pastel tema
  - `2` Jingle Bells: kirmizi-yesil tema
  - `3` Clap Your Hands: mavi-sari tema
- Blink hizi sarki temposuna gore ayarlanir.
- Muzik bittiginde LED durumu muzik oncesindeki haline geri doner.

## Crazy Mode Ozeti

- `4` ile acilip kapanir.
- 17 adimli hareket sekansi uygular.
- ProMaxSpeed kullanilan ileri patlama adimlari vardir.

## Bootloader Referansi

- https://github.com/Robotistan/BerryBot/tree/main/Bootloader

Not: Bootloader islemi oncesinde cihaz modelinin dogru oldugundan emin olun.
