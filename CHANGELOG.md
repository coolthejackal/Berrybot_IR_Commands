# Changelog

Bu dosya projedeki onemli degisiklikleri takip eder.

## [Unreleased]

### Added

- Crazy Mode (`4`) eklendi:
  - 17 adimli hareket sekansi
  - `ProMaxSpeed` kullanilan ileri patlama bolumleri
  - Crazy icon (`zigzag`) gosterimi
- Music Mode eklendi:
  - `*` ile Music Mode'a gecis
  - `#` ile Movement Mode'a donus
  - Music Mode'da motor hareketleri devre disi
  - Matrix'te muzik ikonu gosterimi
- Music Mode melodileri eklendi (3 tekrar):
  - `1`: Happy Birthday To You
  - `2`: Jingle Bells
  - `3`: Clap Your Hands
- Music Mode sirasinda matrix'te secilen rakamin melodi sonuna kadar gosterimi eklendi.
- Muzik sirasinda yan RGB LED'ler icin renkli blink efektleri eklendi.

### Changed

- Raider / Follow / Guard / Sumo / Crazy toggle akislari iyilestirildi.
- `4` tusu davranisi standardize edildi ve Crazy Mode ON/OFF icin duzenlendi.
- Raider acilisinda diger otomatik modlar kapatilacak sekilde mod gecisleri duzenlendi.
- Raider acik yol hareket suresi 3000ms olarak guncellendi.
- IR repeat etkileri, mod ac/kapat tuslarinda daha guvenilir calisacak sekilde guncellendi.
- Sumo mode karar ve durum loglari iyilestirildi (`[SUMO] ...`).

### Improved

- Muzik LED efektleri sarkiya ozel tema renkleri ile zenginlestirildi:
  - `1`: Sicak/pastel
  - `2`: Kirmizi-yesil
  - `3`: Mavi-sari
- LED blink hizi sarki temposuna gore ayarlanir hale getirildi.
- Muzik bitisinde RGB LED durumu muzik oncesindeki state'e geri yuklenecek sekilde duzenlendi.

### Fixed

- Crazy Mode `4` tusu ON/OFF akisinda gorulen tutarsizliklar giderildi.
- Raider Mode ON durumunda bazi gecislerde ortaya cikan mod cakismalari giderildi.
