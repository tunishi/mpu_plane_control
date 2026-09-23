# MPU6050 ile FlightGear Kontrolü (STM32F411 Blackpill)

STM32F411 Blackpill üzerindeki MPU6050'den roll/pitch/yaw okuyup USB seri
üzerinden PC'ye gönderir; PC'de `fg_udp_bridge.py` bu veriyi normalize edip
FlightGear'ın **generic protocol** UDP girişine yazar (aileron/elevator/
rudder property'lerine doğrudan).

```
[MPU6050] --I2C--> [Blackpill] --USB Serial--> [PC: fg_udp_bridge.py] --UDP--> [FlightGear]
```

vJoy + joystick input denendi ama bu kurulumda FlightGear'ın joystick alt
sistemi güvenilir çalışmadı (vJoy eksenleri doğru güncelleniyordu ama
FlightGear'ın kontrol property'lerine hiç yansımıyordu). UDP generic
protocol yöntemi doğrudan property tree'ye yazdığı için çok daha güvenilir
çıktı; proje bu yöntemde sabitlendi. (`vjoy_bridge.py` ve `debug_monitor.py`
hâlâ dursun, ham veriyi izlemek/başka oyunlarla denemek için işe yarar.)

## Hızlı başlangıç

Masaüstündeki iki `.bat` dosyası her şeyi tek tıkla başlatır:

1. **`start_flightgear.bat`** — Cessna 172P'yi LFMD'den 15nm uzakta, 5000ft,
   100kt, motor çalışır ve öğlen vakti (climb config, flaps/gaz doğru
   ayarlı) başlatır. UDP generic protocol dinleyicisi otomatik açılır.
2. Motor tam güce çıkana kadar (~20 saniye) **hiçbir şeye dokunma** —
   kartı da hareket ettirme.
3. **`start_bridge.bat`** — MPU verisini okuyup FlightGear'a gönderir.
   Başlarken kartı otomatik sıfırlar (recenter), elle `c` basmana gerek
   yok.
4. FlightGear penceresinin **3D görünümüne fare ile tıklama** — kokpitteki
   objelerle istemeden etkileşime girip kumandaları kilitleyebiliyor.
   Pencereler arasında geçiş için görev çubuğunu kullan.

## Yaw hakkında önemli not

MPU6050'de manyetometre yok. İvmeölçer sadece yerçekimini görür, bu yüzden
roll ve pitch yerçekimine göre kendini toparlayabiliyor (complementary
filter) ama **yaw'ın hiçbir mutlak referansı yok** — sadece gyro Z
ekseninin zaman içinde integre edilmesiyle elde ediliyor ve zamanla kayar
(drift). Bunu telafi etmek için firmware, seri porttan `c` karakteri
geldiğinde yaw'ı (ve roll/pitch'i) sıfırlar; `fg_udp_bridge.py` çalışırken
konsola `c` yazıp Enter'a basman yeterli (ayrıca script her başladığında
otomatik olarak bir kere recenter gönderir).

İleride gerçek (mutlak) yaw istersen, aynı I2C hattına ucuz bir
manyetometre (ör. QMC5883L/HMC5883L) eklemek yeterli; `firmware/src/main.cpp`
içindeki yaw integrasyonunu pusula okumasıyla değiştirmek küçük bir
değişiklik.

## 1) Donanım bağlantısı

| MPU6050 | Blackpill |
|---------|-----------|
| VCC     | 3V3       |
| GND     | GND       |
| SCL     | PB6       |
| SDA     | PB7       |

Not: test sırasında bir kez I2C hattında gevşeklik yüzünden sensör
gerçek dışı değerler (ör. sabit -135°/35° gibi "magic angle" okumaları)
verdi. Böyle bir şey olursa önce kartın USB'sini çıkarıp takarak güç
kesintisi yap; tekrarlıyorsa jumper kablolarını kontrol et/sağlamlaştır.

## 2) Firmware'i derle ve yükle

```powershell
pip install platformio
cd firmware
pio run -t upload            # USB DFU: BOOT0'a bas, RESET'e bas-bırak, BOOT0'ı bırak
# ST-Link kullanıyorsan:
pio run -e blackpill_f411ce_stlink -t upload
```

Board USB CDC ile takıldığında normal bir COM portu olarak görünür
(şu an COM4). `pio device monitor -p COM4 -b 115200` ile ham
`roll,pitch,yaw` satırlarını görebilirsin, ya da `pc_bridge/debug_monitor.py`
ile etiketli/yavaşlatılmış görünüm.

## 3) PC tarafı: UDP köprü scripti

```powershell
cd pc_bridge
pip install -r requirements.txt
python fg_udp_bridge.py COM4
```

Port belirtmezsen script mevcut portları listeleyip seçmeni ister.

Yön ters çalışıyorsa `fg_udp_bridge.py` içindeki `ROLL_SIGN` /
`PITCH_SIGN` / `YAW_SIGN` değerlerini `1.0` / `-1.0` arasında değiştir
(şu an roll ve pitch ters bulunduğu için `-1.0`, yaw da `-1.0`).

## FlightGear tarafı

`start_flightgear.bat` içindeki `--generic=socket,in,45,,5501,udp,mpu-input`
bayrağı, `$FG_ROOT/Protocol/mpu-input.xml` dosyasını yükler (bu proje
tarafından oraya eklendi). Bu dosya UDP'den gelen `aileron,elevator,rudder`
(virgülle ayrılmış, -1..1 aralığında float) satırlarını doğrudan
`/controls/flight/aileron`, `elevator`, `rudder` property'lerine yazar.

FlightGear'ı elle başlatırsan aynı `--generic=...` ve `--telnet=5500`
(opsiyonel, teşhis için) bayraklarını eklemeyi unutma.

## Kalibrasyon notları

- Firmware açılışta ~1.5 saniye boyunca (LED yanıyorken) gyro bias
  kalibrasyonu yapar — bu sırada MPU6050'yi hareket ettirme.
- `COMPLEMENTARY_ALPHA` (main.cpp) roll/pitch için gyro/ivme ağırlığını
  belirler; 0.98 iyi bir başlangıç değeri.
- `ROLL_RANGE_DEG` / `PITCH_RANGE_DEG` / `YAW_RANGE_DEG` (fg_udp_bridge.py)
  kaç derecelik eğimin tam kontrol sapmasına karşılık geleceğini belirler;
  şu an 45°/45°/180°, uçuş hissine göre ayarlanabilir.
- `SMOOTHING` (fg_udp_bridge.py) ani sıçramaları yumuşatır; 0 = kapalı,
  1'e yaklaştıkça daha yumuşak ama gecikmeli.
