# esp32c3_mxw01

MXW01 BLE thermal printer driver running standalone on an **ESP32-C3 Super Mini**.
No Raspberry Pi, no Python, no host — the C3 is the BLE central, and it also serves
a small web UI so a phone or laptop can send text or a bitmap over WiFi.

```
A2 intensity -> A9 (line count, 0x30, monochrome) -> 48-byte rows on AE03
-> AD flush -> wait for 0xAA print-complete notification
```

## Hardware

Nothing to wire: the printer is BLE, the C3 is powered from USB-C. Keep the
board 1–3 m from the printer and away from metal.

## Build & flash

Arduino IDE:

1. Boards Manager: **esp32 by Espressif** (tested with 3.3.12)
2. Library Manager: **NimBLE-Arduino** by h2zero (tested with 2.5.2)
3. Board: *ESP32C3 Dev Module*, USB CDC On Boot: **Enabled**, Flash 4 MB
4. **Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS)** — WiFi + WebServer push
   the image past 1.2 MB, so the default 1.2 MB app partition no longer fits
5. Flash and open Serial Monitor at **115200**

arduino-cli:

```bash
arduino-cli lib install "NimBLE-Arduino"
arduino-cli compile --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=huge_app" esp32c3_mxw01
arduino-cli upload  -p /dev/cu.usbmodemXXXX \
  --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=huge_app" esp32c3_mxw01
arduino-cli monitor -p /dev/cu.usbmodemXXXX -c baudrate=115200
```

PlatformIO: `pio run -t upload` with the bundled `platformio.ini`.

WiFi credentials live in `secrets.h` (git-ignored; copy `secrets.example.h`). The
sketch also builds without it — it then falls back to its own `MXW01-C3` access
point, so you can flash first and sort out the network later.

## Web UI

Boot log prints the URL:

```
[*] WiFi 'MyNetwork-2.4G' ...
[+] http://192.168.33.129/  (rssi -57)
[*] http server on port 80
```

Open it from any browser on the same network: a text box, an intensity picker,
a PBM upload and a live status panel.

| endpoint | method | body |
|---|---|---|
| `/` | GET | HTML UI |
| `/status` | GET | text: wifi/ip/rssi/battery/temp/fw/queue/intensity |
| `/text?t=...&i=100` | GET | prints text (wrapped, 3x 5x7 font) |
| `/print?i=100` | POST | **base64** of raw 1 bpp rows, 48 B/row, LSB-first |
| `/pbm?i=100` | POST | **base64** of a binary PBM (P4), width ≤ 384 |

Bodies must be base64: the ESP32 `WebServer` swallows any non-form POST body into
a `String`, which truncates at the first `0x00`. It also means the `Content-Type`
must not be `application/x-www-form-urlencoded` — otherwise the body is parsed as
form data and `/print` answers 400.

From the Mac:

```bash
python scripts/send_image_to_c3.py foto.jpg --host 192.168.33.129
python scripts/send_image_to_c3.py --text "merhaba" --host 192.168.33.129
```

The job is queued and the HTTP request returns immediately; the print itself runs
in `loop()`, then WiFi carries on as before. While a page is printing (≈9 s for
300 rows) the web server is blocked, since everything is single-threaded.

## Serial commands

| command | action |
|---|---|
| `p` | status query + built-in self-test page |
| `t <text>` | print text (upper-cased, wrapped to 384 px) |
| `s` | status: state / battery / temperature / error |
| `d` | scan 12 s and dump every advertiser (debugging) |
| `w` | WiFi state, IP, RSSI |
| `c` | toggle "WiFi off during print" (default off) |
| `b` | battery percentage |
| `v` | firmware version |
| `i <0-100>` | print intensity (default 100) |

## Protocol notes

- Frame: `22 21 <cmd> 00 <len_lo> <len_hi> <payload> <crc8(payload)> FF`,
  CRC-8 poly `0x07`, init `0`.
- Row format: 384 px wide, 1 bpp, **LSB-first** (`pixel x -> bit x & 7`). MSB-first
  scrambles the image — that detail cost a debugging session in the Python port.
- **Pace rows to the negotiated connection interval.** The MXW01 negotiates
  30 ms (24 × 1.25 ms). Writing every 6 ms overruns the controller's ~16 TX buffers
  and `writeValue` fails at row 16. One row per interval (30 ms) is reliable:
  300 rows ≈ 9 s, matching the macOS/Bleak timing.
- `0xAA` is the printer's own "job done" notification; the sketch waits up to 90 s
  for it and does not assume success from a clean write.
- The MXW01 reports `AE30` as its service and, unlike the PD01/GT01 family, uses an
  extra `AE03` bulk data characteristic. Connect by name here, so the "needs a BLE
  address, not a name" limitation of the Python/Bleak path does not apply.
- **NimBLE-Arduino 2.x `NimBLEScan::start()` is non-blocking.** It only kicks off the
  scan and returns `bool`; calling `stop()` right after produces a 0 ms scan that
  reports zero devices — which looks exactly like "the printer is not advertising".
  The sketch waits out `isScanning()` before stopping.
- WiFi and BLE share the C3's single 2.4 GHz radio. At interval-paced writes they
  coexist fine; the serial command `c` can still hand the radio over to BLE for the
  duration of a job if you ever see dropped rows.

## Known limits

- Font is a 5x7 bitmap font, `0x20`–`0x5A`; lowercase is upper-cased and Turkish
  letters are folded to ASCII (`ş`→`S`, `ı`→`I`, …) because the font has no accents.
- A page buffer is `48 B × height`, allocated from the heap: 320 rows for text,
  800 rows (38 KB) for uploads. No grayscale on-device — the host does the
  dithering before the POST.
- One printer, one connection at a time. If the vendor phone app is connected, the
  printer stops advertising and the scan will come up empty.
- `secrets.h` is not committed; without it the build fails on the missing include.
