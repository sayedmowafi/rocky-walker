# Rocky Walker: Components

Everything Rocky is made of: what it is, how many, where it goes, and how it connects.
Prices are what the owner actually paid (AED, November 2026, UAE). A rebuilder elsewhere will pay
different prices, but the parts are all common.

**Status key:** ✅ have · 📦 ordered · ⏳ back-ordered · ❓ check you have it

## 1. Motion

| # | Part | Qty | Where it goes | Status | Paid (AED) |
|---|---|---|---|---|---|
| 1 | MG996R servo, Reland Sun | 10 | Hip-yaw (coxa) ×5 and hip-lift (femur) ×5, the high-load joints | 📦 | 217.50 |
| 2 | MG996R servo, Beffkkip (2-pack) | 2 packs = 4 | Knees ×4 | 📦 | 64.00 |
| 3 | MG996R servo (local shop) | 1 | Knee ×1 | 📦 | 21.00 |
| 4 | 25T aluminium round disc horn (5-pack) | 3 packs = 15 | One per servo; the printed brackets bolt to these | 📦 | 59.70 |
| 5 | 623ZZ bearing, 3 × 10 × 4 mm (24-pack) | 15 used, 9 spare | Idler pivot opposite each servo spline | 📦 | 30.44 |
| 6 | PCA9685 16-channel PWM board | 1 | Servo signals from the Pi over I²C (channels 0–14) | 📦 | 28.00 |

Spare servos: none in this order. Spares come from a friend; buy 2 more when stock returns.

## 2. Power

| # | Part | Qty | Where it goes | Status | Paid (AED) |
|---|---|---|---|---|---|
| 7 | Zeee 2S 2200 mAh 50C shorty LiPo, XT60, 73 × 34 × 18.5 mm (2-pack) | 2 | Battery tray under the chassis; one in use, one charging | 📦 | 216.99 |
| 8 | imaxRC B3 PRO 2S/3S balance charger | 1 | Charges through the robot's charging port (needs a UK plug adapter) | 📦 | 27.26 |
| 9 | Inline blade fuse holder, 12 AWG (4-pack, 5–40 A fuses) | 1 used | Battery + lead, **20 A fuse** | 📦 | 21.99 |
| 10 | DMWD 19 mm latching push button, 20 A | 1 | Main power switch (servo + Pi), mounted on the chassis | 📦 | 30.00 |
| 11 | XT60 pairs with 12 AWG pigtails (3 pairs) | 3 | Battery ↔ robot, and the charging port | 📦 | 39.00 |
| 12 | WANGCL UBEC 5/6 V 3 A (5 A peak) | 5 | Belly; **one per leg**, set to **6.0 V**, feeding that leg's servo board | 📦 | 99.50 |
| 13 | XL4005 5 A step-down | 2 (1 spare) | Belly; Pi 5 supply, set to **5.1 V** before connecting the Pi | 📦 | 24.00 |
| 14 | INA219 current/voltage sensor | 1 | Battery voltage monitoring on I²C (low-battery sit-down) | ✅ | — |
| 15 | Duravel 2-core speaker wire, 1.5 mm², 3 m | ~1 m | Power harness inside the chassis | 📦 | 12.00 |
| 16 | sourcing map battery straps, 20 mm × 30 cm (5-pack) | 1–2 | Through the chassis strap slots, around the battery | 📦 | 29.09 |

## 3. Brain, eyes, ears, voice

| # | Part | Qty | Where it goes | Status | Paid (AED) |
|---|---|---|---|---|---|
| 17 | Raspberry Pi 5 (16 GB) | 1 | On the deck under the carapace | ✅ | — |
| 18 | Raspberry Pi Camera Module 3 (standard) | 1 | Camera tab at the front deck edge (legs 0–1) | ✅ | — |
| 19 | Seeed **ReSpeaker Lite**, USB 2-mic array (XU316) | 1 | Microphone + speaker driver; USB to the Pi | ⏳ | ≈ 91 ($24.90) |
| 20 | Seeed Mono Enclosed Speaker, 4 Ω 5 W | 1 | Plugs into the ReSpeaker Lite's SPK connector | ⏳ | ≈ 7 ($2.00) |
| — | Seeed shipping (import duties not included) | — | — | — | ≈ 17 ($4.74) |

## 4. Build materials (already owned)

| Part | Use | Status |
|---|---|---|
| Black PETG (legs), PLA blue/white/black (body, carapace) | All printed parts, about 0.7 kg plus test prints | ✅ |
| M2 / M2.5 / M3 / M4 screw and nut assortment, M3 and M2.5 standoffs | Servo ears, horns, idler bolts, Pi, camera, pillars | ✅ |
| KASTWAVE JST-XH kit (2–7 pin, pre-crimped leads) | All low-current connections (see §6) | ✅ |
| ELEGOO 22 AWG silicone wire kit | Signal wiring and servo lead extensions | ✅ |
| Heat-shrink (580 pcs), solder, flux, wick, hot-air gun | Every joint in the harness | ✅ |
| Vero / perfboard × 6 | Five per-leg servo boards (≈ 30 × 22 mm each, 3 servo headers + power + signal input) | 📦 (AED 30) |
| TPU 95A filament, black | Foot caps (`cad/stl/FootCap.stl`, print 5 + spares) | ❓ (recommended: Bewinner, AED 58.90) |
| Digital calipers, multimeter | Fit coupon, setting the regulators | ✅ |

## 5. Check you have these

| Part | Why | Status |
|---|---|---|
| USB-C ↔ USB-A data cable, 15–20 cm (right-angle ideal) | Pi → ReSpeaker Lite | ❓ |
| Pi 5 camera cable, 22-pin → 15-pin, ~200 mm | The camera's own cable doesn't fit the Pi 5 | ❓ |
| Raspberry Pi 5 Active Cooler | The Pi sits under the carapace | ❓ |
| microSD card, 32 GB+ | Pi operating system | ❓ |
| UK plug adapter | The B3 charger ships with a European plug | ❓ |

## 6. Not used in Rocky

| Part | Why not |
|---|---|
| MakerHawk MAX98357A amp | The ReSpeaker Lite drives the speaker |
| HDMI → USB capture card | The Pi runs headless over Wi-Fi |
| TP4056 charger boards | Single-cell chargers; they can't balance-charge a 2S LiPo |
| ESP32 boards | Kept as the fallback if motion ever moves off the Pi |
| Jumper wires | Replaced by JST-XH and soldered connections (see §7) |

**Spent on Rocky so far: ≈ AED 1,067 (≈ US $290)**, plus Seeed import duty. Not counted: the Pi, the camera and parts already owned.

## 7. Connections: no jumper wires anywhere

| Link | Connector | Wire | Notes |
|---|---|---|---|
| Battery ↔ robot | XT60 | 12 AWG | The battery's own XT60 |
| Battery + → fuse → switch → power split | Soldered and heat-shrunk | 12 AWG pigtails / 1.5 mm² | One soldered junction for +, one for − |
| Power split → each leg UBEC input (×5) | JST-XH 2-pin | 22 AWG | Each UBEC draws under 3 A from the battery side |
| Power split → XL4005 → Pi 5 | JST-XH **4-pin** (2 × 5 V, 2 × GND) | 22 AWG ×2 per pole | Into the Pi's 5 V and GND pins (2/4 and 6/9); two pins per pole share up to 6 A |
| Pi I²C → PCA9685 and INA219 | JST-XH 4-pin (3V3, GND, SDA, SCL), daisy-chained | 22 AWG | 3.3 V logic straight from the Pi |
| Battery voltage → INA219 | JST-XH 2-pin | 22 AWG | Voltage sense only; it doesn't carry servo current |
| PCA9685 channels → servo distribution board | JST-XH 6-pin ×3 (signals 0–14 + GND) | 22 AWG | Signal only |
| Leg UBEC output → that leg's 3 servos | On the distribution board | — | Each leg gets its own 6 V rail; all grounds are common |
| Servo → distribution board | **Servo's own 3-pin plug** | Servo lead (+ extension if short) | One row of 3 headers per leg |
| Pi ↔ ReSpeaker Lite | USB-A ↔ USB-C | Cable | |
| ReSpeaker Lite → speaker | JST (SPK) | Speaker lead | |
| Pi ↔ camera | CSI ribbon (22 → 15 pin) | Ribbon | |
| Charging port | XT60 + JST-XH 3-pin balance extension | 12 AWG / 22 AWG | On the chassis edge |

**Why per-leg servo boards and not straight onto the PCA9685?** The PCA9685's servo headers share **one** power rail fed through thin traces. That's fine for a few servos, but not for 15 servos at up to 15 A. Each leg instead gets a small perfboard between the chassis plates with:
- three 3-pin servo headers (the servos plug straight in);
- that leg's UBEC output on the power row;
- a JST-XH 4-pin input carrying the three signals plus GND from the PCA9685.

The UBECs sit in the belly, one wire pair up to each board.

*(Bench leg: 3 servos on one UBEC is within the PCA9685's limits, so for that test the servos plug directly into the PCA9685.)*
