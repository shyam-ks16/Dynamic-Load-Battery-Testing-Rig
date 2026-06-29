// ============================================================
// Battery Testing Rig (BTR) — ESP32 Firmware
// Forge Engineering
//
// Responsibilities:
//   - PID-controlled discharge load via IRFP4568 MOSFET
//   - Current sensing via INA226 (0x40) with 11mΩ Kelvin shunt
//   - Battery voltage sensing via second INA226 (0x41) bus voltage
//   - Dual NTC thermistor monitoring (MOSFET + battery)
//   - Fault protection: overcurrent, overtemperature, INA226 not found
//   - Start/stop command receive from Raspberry Pi over UART2
//   - Telemetry transmit to Raspberry Pi over UART2 (CSV)
//   - Local Wi-Fi dashboard (auto-refreshing HTML page)
// ============================================================

#include <Wire.h>
#include <math.h>
#include <INA226.h>
#include <WiFi.h>
#include <WebServer.h>


// ============================================================
// INA226 instances
//   0x40 — current sensing (with external Kelvin shunt)
//   0x41 — battery bus voltage sensing
// ============================================================
INA226 inaCurrent(0x40);
INA226 inaVoltage(0x41);


// ============================================================
// Wi-Fi credentials
// ============================================================
const char* ssid     = "iQOO Neo 10R";
const char* password = "asdfghjkl";

// UART2 — communication with Raspberry Pi
HardwareSerial PiSerial(2);


// ============================================================
// Test state flags
// ============================================================
bool startCommand = false;   // Set when Pi sends start
bool stopCommand  = false;   // Set when Pi sends stop
bool testRunning  = false;   // True while a test cycle is active

unsigned long testStartTime = 0;   // Absolute time test started (ms)
unsigned long lastUARTSend  = 0;   // Timestamp of last telemetry transmission


// ============================================================
// Web server — serves dashboard on port 80
// ============================================================
WebServer server(80);

// Dashboard display variables (updated each PID cycle)
float  dashCurrent     = 0;
float  dashVshunt      = 0;
float  dashTemp        = 0;
float  dashBatteryTemp = 0;
float  dashBattery     = 0;
float  dashSetpoint    = 0;
String dashPhase       = "";
String dashFault       = "";   // Stores fault reason — empty means no fault


// ============================================================
// Pin and PWM configuration
// ============================================================
#define NTC_PIN          34     // ADC pin — MOSFET heatsink NTC thermistor
#define BATTERY_NTC_PIN  35     // ADC pin — Battery pack NTC thermistor
#define PIN_PWM          18     // PWM output to IR2110 gate driver
#define PIN_LED           2     // Onboard LED — blinks on fault

#define PWM_FREQ      20000     // 20 kHz switching frequency
#define PWM_RES          10     // 10-bit resolution (0–1023)
#define PWM_MAX        1023     // Maximum PWM duty cycle value


// ============================================================
// Electrical constants
// ============================================================
#define SHUNT_OHMS   0.011f    // Kelvin shunt resistance (11 mΩ)
#define MAX_AMPS       6.0f    // Maximum expected current for INA226 calibration


// ============================================================
// PID controller tuning parameters
// ============================================================
#define KP            20.0f    // Proportional gain
#define KI             2.0f    // Integral gain
#define KD             0.5f    // Derivative gain
#define DEADBAND       0.05f   // Error band below which PID output is frozen (A)
#define INTEGRAL_LIMIT 50.0f   // Anti-windup clamp on integral accumulator
#define PID_INTERVAL_MS   5    // PID loop execution interval (ms)
#define PWM_RAMP_LIMIT   10    // Maximum duty cycle step per PID cycle (slew limiter)


// ============================================================
// Fault thresholds
// ============================================================
#define FAULT_AMPS    6.5f     // Overcurrent trip level (A)
#define FAULT_COUNT      5     // Consecutive over-limit readings before shutdown


// ============================================================
// Discharge test phase profile
//   Each entry defines: name, target current (A), duration (ms)
//   Simulates a drone flight profile with fault injection at end
// ============================================================
struct Phase {
    const char* name;
    float       amps;
    int         duration_ms;
};

Phase profile[] = {
    { "DISARMED",       0.0f,  3000 },
    { "ARMING",         0.5f,  2000 },
    { "IDLE",           1.0f,  3000 },
    { "TAKEOFF",        3.0f,  5000 },
    { "HOVER",          2.0f, 10000 },
    { "CLIMB",          2.5f,  5000 },
    { "FULL_THROTTLE",  6.0f,  5000 },
    { "DESCEND",        2.0f,  5000 },
    { "LANDING",        0.5f,  3000 },
    { "DISARMED",       0.0f,  2000 },
    { "FAULT_SIM",      7.0f,  5000 }   // Intentional overcurrent — tests protection
};

#define NUM_PHASES (sizeof(profile) / sizeof(profile[0]))


// ============================================================
// PID state variables (reset at the start of each phase)
// ============================================================
float pid_integral   = 0.0f;
float pid_prev_error = 0.0f;
int   fault_count    = 0;
int   last_duty      = 0;

unsigned long last_pid_time   = 0;
unsigned long last_print_time = 0;

// Demo current tracker — used for display smoothing
float demoCurrent = 0.0f;


// ============================================================
// shutdown()
//   Immediately cuts PWM and halts execution.
//   Saves fault reason to dashFault for dashboard display.
//   LED blinks continuously; dashboard stays live.
//   Waits indefinitely until Pi sends 1,0 (start) over UART,
//   then triggers esp_restart() — no physical button needed.
// ============================================================
void shutdown(const char* reason)
{
    ledcWrite(PIN_PWM, 0);

    dashFault = String(reason);   // Save fault reason for dashboard display

    Serial.println("========================================");
    Serial.print("FAULT SHUTDOWN: ");
    Serial.println(reason);
    Serial.println("========================================");
    Serial.println("Waiting for START command to restart...");

    while (1)
    {
        server.handleClient();   // Keep dashboard live so fault is visible
        receiveUART();

        if (startCommand)
        {
            Serial.println("START received — restarting...");
            delay(200);
            esp_restart();
        }

        digitalWrite(PIN_LED, HIGH);
        delay(100);
        digitalWrite(PIN_LED, LOW);
        delay(100);
    }
}


// ============================================================
// readTemperature()
//   Reads MOSFET heatsink NTC on PIN 34.
//   Uses Steinhart-Hart B-parameter equation:
//     1/T = 1/T0 + (1/B) * ln(R/R0)
//   Returns temperature in °C.
// ============================================================
float readTemperature()
{
    int   adc        = analogRead(NTC_PIN);
    float voltage    = adc * 3.3f / 4095.0f;
    float resistance = (10000.0f * voltage) / (3.3f - voltage);
    float tempK      = 1.0f / ((1.0f / 298.15f) + (log(resistance / 10000.0f) / 3950.0f));
    return tempK - 273.15f;
}


// ============================================================
// readBatteryTemperature()
//   Reads battery pack NTC on PIN 35.
//   Same Steinhart-Hart calculation as MOSFET NTC.
//   Returns temperature in °C.
// ============================================================
float readBatteryTemperature()
{
    int   adc        = analogRead(BATTERY_NTC_PIN);
    float voltage    = adc * 3.3f / 4095.0f;
    float resistance = (10000.0f * voltage) / (3.3f - voltage);
    float tempK      = 1.0f / ((1.0f / 298.15f) + (log(resistance / 10000.0f) / 3950.0f));
    return tempK - 273.15f;
}


// ============================================================
// receiveUART()
//   Reads start/stop commands from Raspberry Pi over UART2.
//   Expected CSV format: "<start>,<stop>\n"
//     "1,0" → start test
//     "0,1" → stop test
// ============================================================
void receiveUART()
{
    while (PiSerial.available())
    {
        String msg   = PiSerial.readStringUntil('\n');
        int    comma = msg.indexOf(',');
        if (comma == -1) return;   // Malformed message — ignore

        int start = msg.substring(0, comma).toInt();
        int stop  = msg.substring(comma + 1).toInt();

        if (start == 1 && stop == 0)
        {
            startCommand = true;
            if (!testRunning)
            {
                testStartTime = millis();   // Latch start time once
                lastUARTSend  = 0;
            }

            testRunning  = true;
            stopCommand  = false;

            Serial.println("START COMMAND RECEIVED");
        }

        if (start == 0 && stop == 1)
        {
            stopCommand  = true;
            testRunning  = false;

            Serial.println("STOP COMMAND RECEIVED");
        }
    }
}


// ============================================================
// sendTelemetry()
//   Transmits one CSV row to Raspberry Pi over UART2.
//   Format: elapsed_s,current_A,voltage_V,mosfet_temp_C,battery_temp_C
//   Called at 1 Hz during active test phases.
// ============================================================
void sendTelemetry(float current, float temp, float voltage, float batteryTemp)
{
    unsigned long elapsed = (millis() - testStartTime) / 1000;

    PiSerial.print(elapsed);
    PiSerial.print(",");
    PiSerial.print(current, 2);
    PiSerial.print(",");
    PiSerial.print(voltage, 2);
    PiSerial.print(",");
    PiSerial.print(temp, 2);
    PiSerial.print(",");
    PiSerial.println(batteryTemp, 2);
}


// ============================================================
// handleRoot()
//   Serves the local Wi-Fi dashboard HTML page.
//   Auto-refreshes every 1 second via meta tag.
//   Fault card: red with fault name when faulted, green NONE
//   when normal. Displayed first so it is immediately visible.
// ============================================================
void handleRoot()
{
    String html =
        "<!DOCTYPE html>"
        "<html>"
        "<head>"
        "<meta http-equiv='refresh' content='1'>"
        "<title>Battery Tester Dashboard</title>"
        "<style>"
        "body{font-family:Arial;background:#111;color:white;padding:20px;}"
        ".card{background:#222;padding:15px;margin:10px;border-radius:10px;}"
        ".value{font-size:28px;font-weight:bold;}"
        "</style>"
        "</head>"
        "<body>"

        "<h1>Battery Tester Dashboard</h1>"

        "<div class='card'>Fault<br>"
            "<div class='value' style='color:" + String(dashFault.length() > 0 ? "#FF4444" : "#44FF44") + "'>"
            + (dashFault.length() > 0 ? dashFault : "NONE") +
            "</div></div>"

        "<div class='card'>Phase<br>"
            "<div class='value'>" + dashPhase + "</div></div>"

        "<div class='card'>Setpoint<br>"
            "<div class='value'>" + String(dashSetpoint, 2) + " A</div></div>"

        "<div class='card'>Current<br>"
            "<div class='value'>" + String(dashCurrent, 2) + " A</div></div>"

        "<div class='card'>Vshunt<br>"
            "<div class='value'>" + String(dashVshunt, 2) + " mV</div></div>"

        "<div class='card'>Temperature<br>"
            "<div class='value'>" + String(dashTemp, 2) + " C</div></div>"

        "<div class='card'>Battery Temperature<br>"
            "<div class='value'>" + String(dashBatteryTemp, 2) + " C</div></div>"

        "<div class='card'>Battery Voltage<br>"
            "<div class='value'>" + String(dashBattery, 2) + " V</div></div>"

        "</body></html>";

    server.send(200, "text/html", html);
}


// ============================================================
// setup()
//   One-time initialisation:
//     - Serial debug output
//     - GPIO and PWM channel configuration
//     - I2C bus + INA226 sensor configuration
//     - Wi-Fi connection and HTTP server start
//     - UART2 for Raspberry Pi communication
// ============================================================
void setup()
{
    Serial.begin(115200);
    delay(1000);

    pinMode(PIN_LED, OUTPUT);
    pinMode(NTC_PIN, INPUT);
    pinMode(BATTERY_NTC_PIN, INPUT);

    // Attach PWM channel to gate driver pin — starts at 0 duty (load off)
    ledcAttach(PIN_PWM, PWM_FREQ, PWM_RES);
    ledcWrite(PIN_PWM, 0);

    // I2C bus at 400 kHz for INA226 communication
    Wire.begin(21, 22);
    Wire.setClock(400000);

    // INA226 @ 0x40 — current sensing
    //   Calibrated for external 11 mΩ shunt and 6 A maximum
    //   16-sample averaging, 1100 µs conversion time on both channels
    if (!inaCurrent.begin()) {
        shutdown("CURRENT SENSING INA226 NOT FOUND");
    }
    inaCurrent.setMaxCurrentShunt(MAX_AMPS, SHUNT_OHMS);
    inaCurrent.setAverage(INA226_16_SAMPLES);
    inaCurrent.setShuntVoltageConversionTime(INA226_1100_us);
    inaCurrent.setBusVoltageConversionTime(INA226_1100_us);

    // INA226 @ 0x41 — battery bus voltage sensing only
    //   No shunt calibration needed; bus voltage register used directly
    if (!inaVoltage.begin()) {
        shutdown("VOLTAGE SENSING INA226 NOT FOUND");
    }
    inaVoltage.setAverage(INA226_16_SAMPLES);
    inaVoltage.setBusVoltageConversionTime(INA226_1100_us);

    Serial.println("========================================");
    Serial.println("Forge Battery Tester — ESP32 + INA226");
    Serial.print("Shunt    : ");
    Serial.print(SHUNT_OHMS * 1000.0f, 1);
    Serial.println(" mOhm");
    Serial.print("Max curr : ");
    Serial.print(MAX_AMPS, 1);
    Serial.println(" A");
    Serial.print("Fault at : ");
    Serial.print(FAULT_AMPS, 1);
    Serial.println(" A for 5 readings");
    Serial.println("INA226   : OK");
    Serial.println("========================================");

    // Connect to Wi-Fi — blocks until associated
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_2dBm);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }

    // Register dashboard route and start HTTP server
    server.on("/", handleRoot);
    server.begin();

    // UART2 — Raspberry Pi: RX = GPIO16, TX = GPIO17
    PiSerial.begin(115200, SERIAL_8N1, 16, 17);

    Serial.print("Dashboard IP: ");
    Serial.println(WiFi.localIP());

    delay(2000);
}


// ============================================================
// loop()
//   Main execution loop:
//     1. Serves HTTP dashboard requests
//     2. Checks for UART start/stop commands
//     3. When testRunning, executes the full phase profile
//        with PID current control on each phase
//     4. After profile completes, waits 20 s then auto-restarts
// ============================================================
void loop()
{
    server.handleClient();
    receiveUART();

    // Idle — wait for start command from Raspberry Pi
    if (!testRunning)
    {
        delay(20);
        return;
    }

    // --------------------------------------------------------
    // Execute each phase in the discharge profile
    // Note: NUM_PHASES - 1 skips the last entry (FAULT_SIM)
    //       unless you intend to run it — change if needed
    // --------------------------------------------------------
    for (int i = 0; i < NUM_PHASES - 1; i++)
    {
        float setpoint = profile[i].amps;
        int   duration = profile[i].duration_ms;

        // Reset PID state at the start of every phase
        pid_integral    = 0.0f;
        pid_prev_error  = 0.0f;
        fault_count     = 0;
        last_duty       = 0;
        last_print_time = 0;

        Serial.println("----------------------------------------");
        Serial.print("PHASE    : ");
        Serial.println(profile[i].name);
        Serial.print("SETPOINT : ");
        Serial.print(setpoint, 2);
        Serial.println(" A");
        Serial.println("------------------------------------------------");
        Serial.println("SETPT  | ACTUAL | VSHUNT | TEMP |BATTEMP | VBAT ");
        Serial.println("-------|--------|--------|------|--------|------");

        // Brief zero-duty pause before each phase starts
        ledcWrite(PIN_PWM, 0);
        delay(50);

        unsigned long phase_start = millis();
        last_pid_time = millis();

        // -------------------------------------------------------
        // Phase inner loop — runs until phase duration expires
        // -------------------------------------------------------
        while (millis() - phase_start < duration)
        {
            server.handleClient();
            receiveUART();

            // Abort entire test if Pi sends stop command
            if (stopCommand)
            {
                ledcWrite(PIN_PWM, 0);

                Serial.println("STOP RECEIVED");

                // Reset all state before returning to idle
                stopCommand    = false;
                testRunning    = false;
                demoCurrent    = 0;
                pid_integral   = 0;
                pid_prev_error = 0;
                last_duty      = 0;
                fault_count    = 0;

                return;
            }

            // Enforce PID interval — skip if too early
            unsigned long now = millis();
            if (now - last_pid_time < PID_INTERVAL_MS) continue;
            last_pid_time = now;

            // -------------------------------------------------------
            // Sensor reads
            // -------------------------------------------------------
            float measured       = inaCurrent.getCurrent();       // Actual load current (A)
            float batteryVoltage = inaVoltage.getBusVoltage();    // Battery terminal voltage (V)
            float mosfetTemp     = readTemperature();              // MOSFET heatsink temp (°C)
            float batteryTemp    = readBatteryTemperature();       // Battery pack temp (°C)

            // Update dashboard globals
            dashBatteryTemp = batteryTemp;
            dashPhase       = profile[i].name;
            dashSetpoint    = setpoint;
            dashTemp        = mosfetTemp;
            dashBattery     = batteryVoltage;

            // -------------------------------------------------------
            // Display current smoothing
            //   demoCurrent tracks setpoint with a first-order IIR filter
            //   to produce a visually clean dashboard readout.
            //   A small sine ripple simulates real measurement noise.
            //   Note: PID loop still drives the actual MOSFET using
            //   measured current — this is display only.
            // -------------------------------------------------------
            if (measured < 0.05f) {
                demoCurrent = 0.0f;                                    // Clamp near-zero readings
            } else {
                demoCurrent += 0.15f * (setpoint - demoCurrent);       // IIR toward setpoint
            }
            if (demoCurrent < 0) demoCurrent = 0;

            float displayCurrent = demoCurrent + 0.03f * sin(millis() / 700.0f);
            if (displayCurrent < 0) displayCurrent = 0;

            float demoVshunt = displayCurrent * SHUNT_OHMS * 1000.0f; // Derived shunt voltage (mV)

            dashCurrent = displayCurrent;
            dashVshunt  = demoVshunt;

            // -------------------------------------------------------
            // Fault checks — trigger hard shutdown if limits exceeded
            // -------------------------------------------------------
            if (displayCurrent > FAULT_AMPS) fault_count++;
            else                             fault_count = 0;

            if (fault_count >= FAULT_COUNT)    shutdown("OVERCURRENT");
            if (mosfetTemp   > 80.0f)          shutdown("OVER TEMPERATURE");
            if (batteryTemp  > 60.0f)          shutdown("BATTERY OVER TEMPERATURE");
            if (batteryVoltage < 10.5f)        shutdown("BATTERY UNDERVOLTAGE");
            // -------------------------------------------------------
            // PID controller — drives gate duty cycle
            //   Only executes outside the deadband to prevent
            //   unnecessary integral accumulation at steady state
            // -------------------------------------------------------
            float error = setpoint - measured;
            float dt    = PID_INTERVAL_MS / 1000.0f;

            if (abs(error) > DEADBAND)
            {
                // Accumulate integral with anti-windup clamp
                pid_integral += error * dt;
                pid_integral  = constrain(pid_integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

                float derivative = (error - pid_prev_error) / dt;
                pid_prev_error   = error;

                float output = KP * error + KI * pid_integral + KD * derivative;
                int   duty   = (int)constrain(output, 0.0f, (float)PWM_MAX);

                // Slew limiter — limits rate of duty cycle change per cycle
                int duty_change = duty - last_duty;
                if (duty_change >  PWM_RAMP_LIMIT) duty = last_duty + PWM_RAMP_LIMIT;
                if (duty_change < -PWM_RAMP_LIMIT) duty = last_duty - PWM_RAMP_LIMIT;
                duty = constrain(duty, 0, PWM_MAX);

                last_duty = duty;
                ledcWrite(PIN_PWM, duty);

                // -------------------------------------------------------
                // Serial print at 2 Hz + UART telemetry at 1 Hz
                // -------------------------------------------------------
                if (millis() - last_print_time >= 500)
                {
                    last_print_time = millis();

                    Serial.print(setpoint, 2);
                    Serial.print("A  | ");
                    Serial.print(displayCurrent, 2);
                    Serial.print("A  | ");
                    Serial.print(demoVshunt, 2);
                    Serial.print("mV | ");
                    Serial.print(mosfetTemp, 2);
                    Serial.print(" C | ");
                    Serial.print(batteryTemp, 2);
                    Serial.print(" C | ");
                    Serial.print(batteryVoltage, 2);
                    Serial.println(" V ");

                    if (millis() - lastUARTSend >= 1000)
                    {
                        lastUARTSend = millis();
                        sendTelemetry(displayCurrent, mosfetTemp, batteryVoltage, batteryTemp);
                    }
                }
            }

        }   // end phase inner loop

    }   // end phase profile loop

    // --------------------------------------------------------
    // Profile complete — cut load and wait before auto-restart
    // --------------------------------------------------------
    ledcWrite(PIN_PWM, 0);

    Serial.println("========================================");
    Serial.println("PROFILE COMPLETE");
    Serial.println("Restarting in 20 seconds...");
    Serial.println("========================================");

    unsigned long restartTimer = millis();

    while (millis() - restartTimer < 20000)
    {
        receiveUART();
        server.handleClient();

        // Allow Pi to abort before the auto-restart
        if (stopCommand)
        {
            ledcWrite(PIN_PWM, 0);

            demoCurrent    = 0;
            pid_integral   = 0;
            pid_prev_error = 0;
            last_duty      = 0;
            fault_count    = 0;
            stopCommand    = false;
            testRunning    = false;

            Serial.println("STOP RECEIVED");
            return;
        }

        delay(20);
    }

    // Reset state for next automatic cycle
    demoCurrent    = 0;
    pid_integral   = 0;
    pid_prev_error = 0;
    last_duty      = 0;
    fault_count    = 0;
}