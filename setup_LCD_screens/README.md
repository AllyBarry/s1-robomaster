# Waveshare Zero LCD HAT (A) – Custom Script Guide

This guide explains how to set up the Waveshare **Zero LCD HAT (A)** (middle 1.3" ST7789 screen) on a Raspberry Pi running headless, and how to run a custom Python script at boot to display text or graphics.

---

## 1. Enable SPI

The LCD communicates over SPI. Make sure it’s enabled:

```bash
sudo raspi-config nonint do_spi 0
# or manually add:
# dtparam=spi=on
# to /boot/firmware/config.txt (Bookworm) or /boot/config.txt (Legacy)
sudo reboot
```

---

## 2. Install dependencies

```bash
sudo apt update
sudo apt install -y python3-pip python3-pil python3-numpy git
```

---

## 3. Get the demo code / drivers

Clone Waveshare’s demo repo (or use the provided `Zero_LCD_HAT_A_Demo` folder):

```bash
git clone https://github.com/waveshare/LCD-show.git ~/LCD-show
```

For the Zero LCD HAT (A), the relevant drivers live in:

```
Zero_LCD_HAT_A_Demo/python/lib/
```

Look for files like:

```
LCD_1inch3.py   (main 1.3" screen, ST7789)
LCD_0inch96.py  (side screens, ST7735S)
```

---

## 4. Test an example

From the demo folder:

```bash
cd ~/Zero_LCD_HAT_A_Demo/python/example
sudo -E python3 LCD_1inch3_test.py
```

You should see test patterns or text on the middle screen.

---

## 5. Create a custom script

Example: show **"Robo Master 1"** on the middle 1.3" display.

Create `startup_lcd.py` in `~/Zero_LCD_HAT_A_Demo/python/`:

```python
#!/usr/bin/env python3
import time
import spidev as SPI
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
from lib import LCD_1inch3   # driver for the 1.3" ST7789 screen

# Hardware pins (matches Waveshare examples)
RST = 27
DC  = 22
BL  = 19
bus = 1
device = 0

def load_font(base: Path):
    for p in [
        base / "Font" / "Font00.ttf",
        base / "Font" / "Font01.ttf",
        Path("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"),
    ]:
        try:
            return ImageFont.truetype(str(p), 22)
        except Exception:
            pass
    return ImageFont.load_default()

def show(image, disp):
    try:
        disp.ShowImage(image)
    except AttributeError:
        disp.Display(image)

def main():
    base = Path(__file__).parent.resolve()

    disp = LCD_1inch3.LCD_1inch3(
        spi=SPI.SpiDev(bus, device),
        spi_freq=10_000_000,
        rst=RST, dc=DC, bl=BL
    )
    disp.Init()
    disp.clear()
    try:
        disp.bl_DutyCycle(100)
    except AttributeError:
        pass

    W, H = disp.width, disp.height
    img = Image.new("RGB", (W, H), "BLACK")
    draw = ImageDraw.Draw(img)

    text = "Robo Master 1"
    font = load_font(base)

    w, _ = draw.textsize(text, font=font)
    h = font.size + 6
    x, y = (W - w) // 2, (H - h) // 2
    draw.text((x, y), text, fill="WHITE", font=font)

    show(img, disp)

    while True:
        time.sleep(3600)

if __name__ == "__main__":
    main()
```

Make it executable:

```bash
chmod +x ~/Zero_LCD_HAT_A_Demo/python/startup_lcd.py
```

Run manually:

```bash
cd ~/Zero_LCD_HAT_A_Demo/python
sudo -E python3 startup_lcd.py
```

---

### ⚠️ Important: Import style

When writing your own scripts, always import the driver using:

```python
from lib import LCD_1inch3
```

Do **not** try to import it directly like `import LCD_1inch3` — that will cause errors such as:

```
ImportError: attempted relative import with no known parent package
```

This applies to all screens:
- Middle 1.3" screen → `from lib import LCD_1inch3`
- Side 0.96" screens → `from lib import LCD_0inch96`

---

## 6. Run the script on boot

Create a `systemd` service:

```bash
sudo nano /etc/systemd/system/lcd-startup.service
```

Paste:

```ini
[Unit]
Description=LCD Boot Message
Wants=dev-spidev0.0.device
After=multi-user.target dev-spidev0.0.device

[Service]
Type=simple
User=rasppiuser
WorkingDirectory=/home/rasppiuser/Zero_LCD_HAT_A_Demo/python
ExecStart=/usr/bin/python3 /home/rasppiuser/Zero_LCD_HAT_A_Demo/python/startup_lcd.py
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Enable it:

```bash
sudo systemctl daemon-reload
sudo systemctl enable lcd-startup.service
sudo systemctl start lcd-startup.service
```

Check logs if needed:

```bash
journalctl -u lcd-startup.service -f
```

---

## 7. Next steps

- Use the **side screens** (`LCD_0inch96.py`) to display info like IP address or uptime.  
- Customize fonts, colors, or images.  
- Display sensor data or system stats.  

---

✅ With this, you can run any custom Python script on the Zero LCD HAT (A) at boot.
