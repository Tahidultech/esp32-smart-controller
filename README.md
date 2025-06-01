# ESP32 Smart Controller (Arduino IDE)

## Features
- 4 Relay control (ESP RainMaker App and push buttons)
- Fan speed (+/-) via App and dedicated buttons
- IR remote receiver, logs codes to Serial
- LDR sensor: reports light percentage, signal loss counter
- All relays OFF on boot
- Reset button in app to clear signal loss counter

## Pin Mapping

| Feature           | GPIO | Label |
|-------------------|------|-------|
| Light 1 Relay     | 4    | D4    |
| Light 2 Relay     | 5    | D5    |
| Light 3 Relay     | 18   | D18   |
| Fan Relay         | 19   | D19   |
| Fan Speed +       | 21   | D21   |
| Fan Speed -       | 22   | D22   |
| Button Light1     | 23   | D23   |
| Button Light2     | 25   | D25   |
| Button Light3     | 26   | D26   |
| Button Fan        | 27   | D27   |
| Button Fan +      | 32   | D32   |
| Button Fan -      | 33   | D33   |
| IR Receiver       | 15   | D15   |
| LDR Sensor        | 34   | D34   |

## Flashing

See `FLASHING_INFO.txt` for details and partition/bootloader offsets.

## Libraries Needed

- [ESP32 RainMaker Arduino Library](https://github.com/espressif/arduino-esp32/tree/master/libraries/RainMaker)
- [IRremoteESP8266](https://github.com/crankyoldgit/IRremoteESP8266)

## GitHub Actions

- `.github/workflows/build.yml` — Arduino build check on push/PR
- `.github/workflows/release.yml` — Compile and upload bin on tag or manual

## Notes

- LDR is input only (GPIO34)
- All relays are OFF by default at boot
- You must provision WiFi/RainMaker credentials as per Espressif docs
