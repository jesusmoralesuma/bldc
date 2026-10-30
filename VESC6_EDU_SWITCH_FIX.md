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

### Electrical & Firmware Mechanism
1. **Dual-Role Pin Multiplexing (`PC5` / `ADC1_IN15`):**
   On the VESC 6 hardware architecture, microcontroller pin `PC5` performs dual roles:
   * **Digital Output (Power Latch):** Driven high (`HW_SHUTDOWN_HOLD_ON()`) to bias an external gatekeeper FET and maintain power to the internal buck regulators (3.3V / 5V rails).
   * **Analog Input (Switch Sampling):** Periodically toggled by `hw_sample_shutdown_button()` to `PAL_MODE_INPUT_ANALOG` for 3 consecutive cycles (0.5 ms settling + 1.0 ms decay measurement + 1.0 ms recharge) to measure voltage decay across the switch/capacitor circuit using the internal ADC (`ADC_IND_SHUTDOWN`).

2. **Signal Polarity of `hw_sample_shutdown_button()`:**
   * **Switch CLOSED to GND (Normal Run State / Button Untouched):** While `PC5` is switched to `PAL_MODE_INPUT_ANALOG`, the closed NC switch discharges the sensing capacitor toward GND over the 1 ms window (`val1 > val2`), producing a positive cumulative voltage drop `bt_diff > 0.12V`. Thus, **`HW_SAMPLE_SHUTDOWN()` returns `true` (`1`) during normal operation**.
   * **Switch OPEN (Button Pressed / Shutdown Requested):** Opening the NC switch interrupts the discharge path (`val1 ≈ val2`, `bt_diff <= 0.12V`). Thus, **`HW_SAMPLE_SHUTDOWN()` returns `false` (`0`) when a shutdown is requested**.

3. **Ground Bounce & Phase PWM EMI Under Load:**
   Motor phases switch at 20–30 kHz PWM frequencies with rapid current transitions ($\frac{di}{dt}$). The switching current returning through the low-side shunt resistors creates high-frequency ground bounce between power GND and the microcontroller's analog/digital ground. An external switch cable acts as an inductive antenna picking up phase noise.
   During the 1 ms window between `val1` and `val2`, an EMI spike or upward shift in the external switch GND potential can cause `val2 > val1` (a negative `val1 - val2` delta) or momentarily prevent discharge. In the stock firmware, `bt_diff += (val1 - val2)` sums raw signed differences across only 3 cycles, so a single negative noise spike cancels out the positive discharge of the other cycles and drops `bt_diff` below `0.12V`, causing `hw_sample_shutdown_button()` to return `false`.

4. **Fatal Flaw in State Machine Debouncing (`shutdown.c`):**
   In `shutdown_thread()`:
   ```c
   bool sample = HW_SAMPLE_SHUTDOWN();
   bool clicked = m_button_pressed && !sample;
   m_button_pressed = sample;
   ```
   There was **zero software debouncing**. During normal operation (`m_button_pressed == true`), **the very first loop iteration** where `HW_SAMPLE_SHUTDOWN()` returns `false` due to an EMI transient causes `clicked = m_button_pressed && !sample` (`true && !false`) to immediately evaluate to `true`. Under the default configuration (`SHUTDOWN_MODE_OFF_AFTER_30M` or `SHUTDOWN_MODE_TOGGLE_BUTTON_ONLY`), `clicked` immediately invokes `do_shutdown(false)`, which cuts the gate drivers (`DISABLE_GATE()`) and drops the power hold line (`HW_SHUTDOWN_HOLD_OFF()`), causing an instantaneous power blackout.

5. **Absence of Motor-Running Interlock:**
   The stock shutdown sequence never checked motor rotational speed (`mc_interface_get_rpm()`). Any single false sample under load immediately killed power mid-operation.

---

## 2. Comprehensive Mitigation Strategies

### A. Firmware Fix (The Permanent Solution)
Implemented in this repository on branch `fix/vesc6-edu-nc-switch-emi`:
* **Consecutive-Sample Debounce Integrator (`hwconf/shutdown.c`):**
  * Requires `5` consecutive `true` samples (~88 ms) to arm the normal closed state (`debounced_closed = true`).
  * When the motor is stopped (`|ERPM| <= 200`), requires `18` consecutive `false` samples (~315 ms of continuous switch opening) before transitioning `debounced_closed` to `false` and triggering `clicked`.
* **Motor Running Safety Interlock & Emergency Hold:**
  * While the motor is spinning (`|ERPM| > 200`), transient EMI bursts and short switch openings are completely ignored.
  * To allow intentional emergency shutdown while the motor is spinning, the switch must remain continuously open (`sample == false`) for `170` consecutive cycles (~3.0 seconds).
* **Negative EMI Spike Rejection (`hw_edu_core.c` & `hw_60_core.c`):**
  * Clamps negative `val1 - val2` noise spikes (`if (diff > 0.0f) bt_diff += diff;`) so a single ground-bounce spike on `val2` cannot cancel out valid capacitor discharge measurements from the other cycles, while preserving the stock `chThdSleep(5)` (500 µs) settling time and `0.12V` RC decay threshold.

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
index 07ef167..51a94fa 100644
--- a/hwconf/shutdown.c
+++ b/hwconf/shutdown.c
@@ -21,6 +21,7 @@
 #include "app.h"
 #include "conf_general.h"
 #include "mc_interface.h"
+#include <math.h>
 
 #ifdef USE_LISPBM
 #include "lispif.h"
@@ -33,7 +34,15 @@
 // Private variables
 bool volatile m_button_pressed = false;
 static volatile float m_inactivity_time = 0.0;
-static THD_WORKING_AREA(shutdown_thread_wa, 128);
+// Debounce settings for noise rejection (~17.5ms per loop iteration)
+// Note: HW_SAMPLE_SHUTDOWN() returns true when NC switch is CLOSED (normal run state)
+// and false when NC switch is OPEN (button pressed / shutdown requested).
+#define SHUTDOWN_CLOSE_DEBOUNCE_SAMPLES     5    // ~88ms continuous closed to arm normal state
+#define SHUTDOWN_OPEN_DEBOUNCE_SAMPLES     18    // ~315ms continuous open to trigger shutdown when stopped
+#define SHUTDOWN_EMERGENCY_OPEN_SAMPLES   170    // ~3.0s continuous open for emergency shutdown while running
+#define SHUTDOWN_ERPM_RUNNING_THRESHOLD   200.0f // ERPM threshold to consider motor spinning
+
+static THD_WORKING_AREA(shutdown_thread_wa, 256);
 static mutex_t m_sample_mutex;
 static volatile bool m_init_done = false;
 static volatile bool m_sampling_disabled = false;
@@ -125,8 +134,35 @@ static THD_FUNCTION(shutdown_thread, arg) {
 
 		bool sample = HW_SAMPLE_SHUTDOWN();
 		chMtxUnlock(&m_sample_mutex);
-		bool clicked = m_button_pressed && !sample;
-		m_button_pressed = sample;
+
+		bool motor_is_running = (fabsf(mc_interface_get_rpm()) > SHUTDOWN_ERPM_RUNNING_THRESHOLD);
+		int required_open_samples = motor_is_running ?
+				SHUTDOWN_EMERGENCY_OPEN_SAMPLES : SHUTDOWN_OPEN_DEBOUNCE_SAMPLES;
+
+		static int open_counter = 0;
+		static int close_counter = 0;
+		static bool debounced_closed = false;
+
+		if (sample) {
+			if (close_counter < SHUTDOWN_CLOSE_DEBOUNCE_SAMPLES) {
+				close_counter++;
+			}
+			open_counter = 0;
+			if (close_counter >= SHUTDOWN_CLOSE_DEBOUNCE_SAMPLES) {
+				debounced_closed = true;
+			}
+		} else {
+			if (open_counter < SHUTDOWN_EMERGENCY_OPEN_SAMPLES) {
+				open_counter++;
+			}
+			close_counter = 0;
+			if (open_counter >= required_open_samples) {
+				debounced_closed = false;
+			}
+		}
+
+		bool clicked = m_button_pressed && !debounced_closed;
+		m_button_pressed = debounced_closed;
 
 		const app_configuration *conf = app_get_configuration();
 
@@ -136,7 +172,7 @@ static THD_FUNCTION(shutdown_thread, arg) {
 
 		switch (conf->shutdown_mode) {
 		case SHUTDOWN_MODE_ALWAYS_OFF:
-			if (m_button_pressed) {
+			if (m_button_pressed && !motor_is_running) {
 				gates_disabled_here = do_shutdown(true);
 			}
 			break;
@@ -191,7 +227,7 @@ static THD_FUNCTION(shutdown_thread, arg) {
 			default: break;
 			}
 
-			if (m_inactivity_time >= shutdown_timeout && m_button_pressed) {
+			if (m_inactivity_time >= shutdown_timeout && m_button_pressed && !motor_is_running) {
 				gates_disabled_here = do_shutdown(false);
 			}
 		}
diff --git a/hwconf/trampa/vesc6/hw_60_core.c b/hwconf/trampa/vesc6/hw_60_core.c
index edf5809..c6835ee 100644
--- a/hwconf/trampa/vesc6/hw_60_core.c
+++ b/hwconf/trampa/vesc6/hw_60_core.c
@@ -322,7 +322,10 @@ bool hw_sample_shutdown_button(void) {
 		palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_OUTPUT_PUSHPULL);
 		chThdSleepMilliseconds(1);
 
-		bt_diff += (val1 - val2);
+		float diff = val1 - val2;
+		if (diff > 0.0f) {
+			bt_diff += diff;
+		}
 	}
 
 	chMtxUnlock(&shutdown_mutex);
diff --git a/hwconf/trampa/vesc_edu/hw_edu_core.c b/hwconf/trampa/vesc_edu/hw_edu_core.c
index 37f830a..2829cb0 100644
--- a/hwconf/trampa/vesc_edu/hw_edu_core.c
+++ b/hwconf/trampa/vesc_edu/hw_edu_core.c
@@ -285,7 +285,10 @@ bool hw_sample_shutdown_button(void) {
 		palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_OUTPUT_PUSHPULL);
 		chThdSleepMilliseconds(1);
 
-		bt_diff += (val1 - val2);
+		float diff = val1 - val2;
+		if (diff > 0.0f) {
+			bt_diff += diff;
+		}
 	}
 
 	chMtxUnlock(&shutdown_mutex);
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
   make fw_edu_clean
   ```
3. Compile the target:
   ```bash
   make edu
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
3. When the NC switch is closed (normal operation), `BT:` stays at `1 <bt_diff>` (with `bt_diff > 0.12`).
4. When the NC switch is opened (shutdown requested), `BT:` drops to `0 <bt_diff>` (with `bt_diff <= 0.12`).
5. Run the motor under high load: transient EMI spikes will no longer trigger an unintentional power-off.
6. Stop the motor and open the switch for >0.3 seconds: the VESC will perform a clean, controlled power-off.
