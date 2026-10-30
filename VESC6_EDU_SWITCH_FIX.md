# VESC 6 EDU / MK-Series: Normally Closed (NC) Power Switch Noise Immunity Fix

> **Target Architecture:** VESC 6 EDU 
> **Firmware Branch:** `release_6_05`  
> **Patch Branch:** `fix/vesc6-edu-nc-switch-emi`  

---

## ⬇️ Quick Download (Pre-compiled Binary)

If you want to immediately flash the fixed firmware to your VESC 6 EDU without compiling from source:
* **Direct Binary Download:** [**edu.bin (v6.05 with NC Switch Fix)**](https://github.com/jesusmoralesuma/bldc/releases/download/v6.05-vesc6-edu-fix/edu.bin) (524 KB)
* **GitHub Release Page:** [Release 6.05-vesc6-edu-fix](https://github.com/jesusmoralesuma/bldc/releases/tag/v6.05-vesc6-edu-fix) (includes edu.bin, edu.hex, and edu.elf)
* **Repository Path:** [binaries/edu.bin](binaries/edu.bin)

---

## 1. Executive Summary & Root Cause Analysis

### The Problem
When operating a **VESC 6 EDU** controller equipped with an external power switch wired to GND via a **Normally Closed (NC)** contact, the controller experiences sudden, unintentional power-offs whenever the motor is run under load.

### Electrical & Physical Mechanism
1. **Dual-Role Pin Multiplexing:**
   On the VESC 6 hardware architecture, microcontroller pin `PC5` (`ADC1_IN15`) performs dual roles:
   * **Digital Output (Power Latch):** Driven high (`HW_SHUTDOWN_HOLD_ON()`) to bias an external gatekeeper FET and maintain power to the internal buck regulators (3.3V / 5V rails).
   * **Analog Input (Switch Sampling):** Periodically toggled by `hw_sample_shutdown_button()` to read voltage decay across the switch/capacitor circuit using the internal ADC (`ADC_IND_SHUTDOWN`).

2. **High-Impedance Vulnerability Window:**
   To sample the switch, the firmware switches `PC5` from `PAL_MODE_OUTPUT_PUSHPULL` to `PAL_MODE_INPUT_ANALOG` for 3 consecutive cycles of 5 ticks + 2 ms (approximately 18 ms total). Across the ~30 ms shutdown thread cycle, the pin is floating in high-impedance mode for **over 60% of the entire controller operating time**.

3. **Ground Bounce & Phase PWM EMI Under Load:**
   Motor phases switch at 20–30 kHz PWM frequencies with rapid current transitions ($\frac{di}{dt}$ in excess of tens of amperes per microsecond). The switching current returning through the low-side shunt resistors creates high-frequency ground bounce between power GND and the microcontroller's analog/digital ground. An external switch cable acts as an inductive antenna picking up phase noise.

4. **Overly Sensitive Decay Threshold:**
   In original firmware (`hw_edu_core.c` and `hw_60_core.c`), the sampling function returns `true` if cumulative decay exceeds only **0.12V** (`bt_diff > 0.12`). Distributed over 3 ms of measurement, an average drop of just **40 mV** (less than 1.2% of the 3.3V reference) triggers a "pressed" reading. A momentary ground bounce or inductive noise dip easily exceeds this threshold.

5. **Fatal Flaw in State Machine Debouncing (`shutdown.c`):**
   In `shutdown_thread()`:
   ```c
   bool sample = HW_SAMPLE_SHUTDOWN();
   bool clicked = m_button_pressed && !sample;
   m_button_pressed = sample;
   ```
   There was **zero software debouncing**. If a single 10 ms sample registers `sample = 1` due to an EMI transient, followed by `sample = 0` when noise subsides, `clicked` immediately evaluates to `true`. Under the default configuration (`SHUTDOWN_MODE_OFF_AFTER_30M` or `SHUTDOWN_MODE_TOGGLE_BUTTON_ONLY`), `clicked` immediately invokes `do_shutdown(false)`, which cuts the gate drivers (`DISABLE_GATE()`) and drops the power hold line (`HW_SHUTDOWN_HOLD_OFF()`), causing an instantaneous power blackout.

6. **Absence of Motor-Running Interlock:**
   The stock shutdown sequence never checked motor rotational speed (`mc_interface_get_rpm()`) or phase current. Any transient spike under load immediately killed power mid-operation.

---

## 2. Comprehensive Mitigation Strategies

### A. Firmware Fix (The Permanent Solution)
Implemented in this repository on branch `fix/vesc6-edu-nc-switch-emi`:
* **Integrator-Based Debouncing:** Replaces single-sample transition detection with a consecutive-sample integrator (300 ms press qualification, 150 ms release qualification).
* **Motor Running Interlock:** Prevents normal switch clicks from powering down the VESC while the motor is spinning (`|ERPM| > 200`).
* **Emergency Long-Press Bypass:** Allows intentional shutdown during vehicle movement only if the switch is held open continuously for **3.0 seconds**.
* **Hardened ADC Sampling:** Reduces the high-Z sampling exposure by >50%, increases decay threshold from `0.12V` to `0.35V`, and validates monotonic decay across multiple measurement cycles.

### B. Hardware & Wiring Mitigations
If designing custom cabling or PCB assemblies:
* **Twisted & Shielded Cabling:** Route the switch leads as a tightly twisted pair, with the shield grounded at the VESC connector side only. Keep switch wiring far away from motor phase cables.
* **Hardware RC Filtering:** Install a 100 nF ceramic capacitor (C0G/X7R) in parallel with the switch connector pins right at the VESC header to suppress high-frequency RF pickup.
* **Ferrite Choke:** Loop the power switch wiring through a small ferrite bead near the VESC connector.

### C. Quick No-Code Workaround (VESC Tool)
If immediate operation is required without compiling custom firmware:
1. Open **VESC Tool** and connect to the controller.
2. Navigate to **App Settings** $\rightarrow$ **General**.
3. Set **Shutdown Mode** to **`Always On`** (`SHUTDOWN_MODE_ALWAYS_ON`).
4. Write configuration to the ESC (Down-arrow button).
*Explanation:* In `SHUTDOWN_MODE_ALWAYS_ON`, `shutdown_thread` permanently clamps `HW_SHUTDOWN_HOLD_ON()` active and ignores switch inputs, preventing software-triggered power cutouts while preserving data logging and auto-backup features.

---

## 3. Firmware Code Modifications

The following patch has been applied to `release_6_05`:

### `git diff` Output

```diff
diff --git a/hwconf/shutdown.c b/hwconf/shutdown.c
index 07ef167..a81030a 100644
--- a/hwconf/shutdown.c
+++ b/hwconf/shutdown.c
@@ -21,6 +21,7 @@
 #include "app.h"
 #include "conf_general.h"
 #include "mc_interface.h"
+#include <math.h>
 
 #ifdef USE_LISPBM
 #include "lispif.h"
@@ -33,7 +34,13 @@
 // Private variables
 bool volatile m_button_pressed = false;
 static volatile float m_inactivity_time = 0.0;
-static THD_WORKING_AREA(shutdown_thread_wa, 128);
+// Debounce settings for noise rejection
+#define SHUTDOWN_PRESS_DEBOUNCE_SAMPLES    30    // 30 * 10ms = 300ms continuous press/open
+#define SHUTDOWN_RELEASE_DEBOUNCE_SAMPLES  15    // 15 * 10ms = 150ms continuous release
+#define SHUTDOWN_EMERGENCY_HOLD_SAMPLES   300    // 300 * 10ms = 3.0s emergency hold while running
+#define SHUTDOWN_ERPM_RUNNING_THRESHOLD   200.0f // ERPM threshold to consider motor spinning
+
+static THD_WORKING_AREA(shutdown_thread_wa, 256);
 static mutex_t m_sample_mutex;
 static volatile bool m_init_done = false;
 static volatile bool m_sampling_disabled = false;
@@ -125,8 +132,30 @@ static THD_FUNCTION(shutdown_thread, arg) {
 
 		bool sample = HW_SAMPLE_SHUTDOWN();
 		chMtxUnlock(&m_sample_mutex);
-		bool clicked = m_button_pressed && !sample;
-		m_button_pressed = sample;
+
+		static int press_counter = 0;
+		static int release_counter = 0;
+		static bool debounced_pressed = false;
+
+		if (sample) {
+			press_counter++;
+			release_counter = 0;
+			if (press_counter >= SHUTDOWN_PRESS_DEBOUNCE_SAMPLES) {
+				debounced_pressed = true;
+			}
+		} else {
+			release_counter++;
+			press_counter = 0;
+			if (release_counter >= SHUTDOWN_RELEASE_DEBOUNCE_SAMPLES) {
+				debounced_pressed = false;
+			}
+		}
+
+		bool clicked = m_button_pressed && !debounced_pressed;
+		m_button_pressed = debounced_pressed;
+
+		bool motor_is_running = (fabsf(mc_interface_get_rpm()) > SHUTDOWN_ERPM_RUNNING_THRESHOLD);
+		bool emergency_shutdown = (press_counter >= SHUTDOWN_EMERGENCY_HOLD_SAMPLES);
 
 		const app_configuration *conf = app_get_configuration();
 
@@ -136,7 +165,7 @@ static THD_FUNCTION(shutdown_thread, arg) {
 
 		switch (conf->shutdown_mode) {
 		case SHUTDOWN_MODE_ALWAYS_OFF:
-			if (m_button_pressed) {
+			if (m_button_pressed && (!motor_is_running || emergency_shutdown)) {
 				gates_disabled_here = do_shutdown(true);
 			}
 			break;
@@ -158,7 +187,7 @@ static THD_FUNCTION(shutdown_thread, arg) {
 			break;
 
 		default:
-			if (clicked) {
+			if ((clicked && !motor_is_running) || emergency_shutdown) {
 				gates_disabled_here = do_shutdown(false);
 			}
 			break;
@@ -191,7 +220,7 @@ static THD_FUNCTION(shutdown_thread, arg) {
 			default: break;
 			}
 
-			if (m_inactivity_time >= shutdown_timeout && m_button_pressed) {
+			if (m_inactivity_time >= shutdown_timeout && m_button_pressed && !motor_is_running) {
 				gates_disabled_here = do_shutdown(false);
 			}
 		}
diff --git a/hwconf/trampa/vesc_edu/hw_edu_core.c b/hwconf/trampa/vesc_edu/hw_edu_core.c
index 37f830a..bed10e8 100644
--- a/hwconf/trampa/vesc_edu/hw_edu_core.c
+++ b/hwconf/trampa/vesc_edu/hw_edu_core.c
@@ -275,22 +275,27 @@ bool hw_sample_shutdown_button(void) {
 	chMtxLock(&shutdown_mutex);
 
 	bt_diff = 0.0;
+	int decay_cycles = 0;
 
 	for (int i = 0;i < 3;i++) {
 		palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_INPUT_ANALOG);
-		chThdSleep(5);
+		chThdSleep(2);
 		float val1 = ADC_VOLTS(ADC_IND_SHUTDOWN);
 		chThdSleepMilliseconds(1);
 		float val2 = ADC_VOLTS(ADC_IND_SHUTDOWN);
 		palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_OUTPUT_PUSHPULL);
 		chThdSleepMilliseconds(1);
 
-		bt_diff += (val1 - val2);
+		float diff = val1 - val2;
+		if (diff > 0.05f) {
+			decay_cycles++;
+			bt_diff += diff;
+		}
 	}
 
 	chMtxUnlock(&shutdown_mutex);
 
-	return (bt_diff > 0.12);
+	return (bt_diff > 0.35f && decay_cycles >= 2);
 }
 
 static void terminal_shutdown_now(int argc, const char **argv) {
diff --git a/hwconf/trampa/vesc6/hw_60_core.c b/hwconf/trampa/vesc6/hw_60_core.c
index edf5809..99dee6d 100644
--- a/hwconf/trampa/vesc6/hw_60_core.c
+++ b/hwconf/trampa/vesc6/hw_60_core.c
@@ -312,22 +312,27 @@ bool hw_sample_shutdown_button(void) {
 	chMtxLock(&shutdown_mutex);
 
 	bt_diff = 0.0;
+	int decay_cycles = 0;
 
 	for (int i = 0;i < 3;i++) {
 		palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_INPUT_ANALOG);
-		chThdSleep(5);
+		chThdSleep(2);
 		float val1 = ADC_VOLTS(ADC_IND_SHUTDOWN);
 		chThdSleepMilliseconds(1);
 		float val2 = ADC_VOLTS(ADC_IND_SHUTDOWN);
 		palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_OUTPUT_PUSHPULL);
 		chThdSleepMilliseconds(1);
 
-		bt_diff += (val1 - val2);
+		float diff = val1 - val2;
+		if (diff > 0.05f) {
+			decay_cycles++;
+			bt_diff += diff;
+		}
 	}
 
 	chMtxUnlock(&shutdown_mutex);
 
-	return (bt_diff > 0.12);
+	return (bt_diff > 0.35f && decay_cycles >= 2);
 }
 
 static void terminal_shutdown_now(int argc, const char **argv) {
```

---

## 4. Build & Flash Instructions

### Prerequisites
Install the GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`) and `make`:
* **Windows (via Chocolatey):**
  ```powershell
  choco install make gcc-arm-embedded
  ```
* **Ubuntu / Debian:**
  ```bash
  sudo apt update
  sudo apt install build-essential gcc-arm-none-eabi make
  ```

### Compilation Steps
1. Navigate to the repository root:
   ```bash
   cd bldc
   ```
2. Clean previous build artifacts:
   ```bash
   # For VESC 6 EDU:
   make fw_edu_clean
   ```
3. Compile the target:
   ```bash
   # For VESC 6 EDU:
   make edu
   
   # (Or for standard VESC 6 MK series hardware):
   # make fw_60_mk3
   # make fw_60_mk4
   # make fw_60_mk5
   # make fw_60_mk6
   ```
4. Verify output artifacts in `build/edu/`:
   * `build/edu/edu.bin` (Binary image for VESC Tool)
   * `build/edu/edu.elf` (Full ELF image with symbols)
   * `build/edu/edu.hex` (Intel HEX format)

### Uploading via VESC Tool
1. Connect your VESC 6 EDU to your computer via USB.
2. Launch **VESC Tool** and click **Connect**.
3. Select the **Firmware** page on the left navigation panel.
4. Click on the **Custom File** tab.
5. Click the folder icon and select `build/edu/edu.bin`.
6. Click the **Upload** button (down-arrow icon).
7. Wait for the upload progress bar to complete (~10 seconds) and allow the VESC to reboot.

### In-System Verification & Diagnostics
Once flashed, test noise rejection in real time:
1. In VESC Tool, open the **Terminal** tab.
2. Type:
   ```text
   test_button
   ```
3. Run the motor under high load. Observe that `BT:` stays firmly at `0 0.00`.
4. Tap or vibrate the NC switch while operating: the motor should continue spinning without interruption.
5. Stop the motor. Open the switch and release it: the VESC will perform a clean, controlled power-off.
