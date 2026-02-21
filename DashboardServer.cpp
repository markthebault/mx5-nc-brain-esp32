#include "DashboardServer.h"
#include "Config.h"
#include "Debug.h"
#include <WiFi.h>
#include <NetworkClient.h>
#include <WiFiAP.h>

// --- Module State ---
static NetworkServer server(WebServerConfig::PORT);
static TelemetryData* pData = nullptr;

// --- Forward Declarations ---
static void serverTask(void* pvParameters);
static void handleClient(NetworkClient& client);
static String getRequestPath(NetworkClient& client);
static void sendJsonResponse(NetworkClient& client);
static void sendHtmlResponse(NetworkClient& client);

// --- DashboardServer Namespace Implementation ---

namespace DashboardServer {

    void init() {
        server.begin();
        DEBUG_PRINTF("Dashboard server started on port %d\n", WebServerConfig::PORT);
        DEBUG_PRINTF("Connect to WiFi '%s' and open http://%s\n",
                     WebServerConfig::AP_SSID,
                     WiFi.softAPIP().toString().c_str());
    }

    void startTask(TelemetryData* telemetryData) {
        pData = telemetryData;
        xTaskCreate(serverTask, "dashboard_server", WebServerConfig::TASK_STACK_SIZE,
                    nullptr, 1, nullptr);
        DEBUG_PRINTLN("Dashboard server task started.");
    }

}

// --- FreeRTOS Task ---

static void serverTask(void* pvParameters) {
    for (;;) {
        NetworkClient client = server.accept();
        if (client) {
            handleClient(client);
            client.stop();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// --- HTTP Handling ---

static void handleClient(NetworkClient& client) {
    String path = getRequestPath(client);

    if (path == "/api/data") {
        sendJsonResponse(client);
    } else {
        sendHtmlResponse(client);
    }
}

static String getRequestPath(NetworkClient& client) {
    String path = "/";
    unsigned long timeout = millis() + 1000;

    while (client.connected() && millis() < timeout) {
        if (client.available()) {
            String line = client.readStringUntil('\n');
            // Parse first line: "GET /path HTTP/1.1"
            if (line.startsWith("GET ")) {
                int pathEnd = line.indexOf(' ', 4);
                if (pathEnd > 4) {
                    path = line.substring(4, pathEnd);
                }
            }
            // Consume remaining headers until blank line
            if (line == "\r") break;
        }
    }
    return path;
}

static void sendJsonResponse(NetworkClient& client) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Connection: close");
    client.println();

    client.printf(
        "{\"oilTemp\":%.2f,\"waterTemp\":%.2f,\"engineRPM\":%u,"
        "\"oilPressure\":%.2f,\"brakePressure\":%.2f,\"brakePercent\":%d,"
        "\"throttlePos\":%.1f,\"speed\":%.2f,\"accelPos\":%.1f,"
        "\"gaugeType\":\"%s\",\"luminosity\":%u,\"uptime\":%lu}",
        pData->oilTemp, pData->waterTemp, pData->engineRPM,
        pData->oilPressure, pData->brakePressure, pData->brakePercent,
        pData->throttlePos, pData->speed, pData->accelPos,
        pData->gaugeType == GAUGE_RACING ? "RACING" : "NORMAL",
        pData->luminosity, millis() / 1000
    );
}

static void sendHtmlResponse(NetworkClient& client) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();

    client.print(
        "<!DOCTYPE html><html><head>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>MX5 NC Dashboard</title>"
        "<style>"
        "*{box-sizing:border-box;margin:0;padding:0}"
        "body{font-family:system-ui,sans-serif;background:#1a1a2e;color:#e0e0e0;padding:16px}"
        "h1{text-align:center;color:#0ff;font-size:1.4em;margin-bottom:12px}"
        ".grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;max-width:500px;margin:0 auto}"
        ".card{background:#16213e;border-radius:10px;padding:14px;text-align:center;"
        "border:1px solid #0f3460}"
        ".label{font-size:.75em;color:#888;text-transform:uppercase;letter-spacing:1px}"
        ".value{font-size:1.8em;font-weight:700;color:#0ff;margin:4px 0}"
        ".unit{font-size:.7em;color:#666}"
        ".gauge-mode{color:#f0c040}"
        ".status{text-align:center;margin-top:12px;font-size:.75em;color:#555}"
        "</style></head><body>"
        "<h1>MX5 NC Telemetry</h1>"
        "<div class='grid'>"
        "<div class='card'><div class='label'>Engine RPM</div>"
        "<div class='value' id='rpm'>--</div></div>"
        "<div class='card'><div class='label'>Speed</div>"
        "<div class='value' id='speed'>--</div><div class='unit'>km/h</div></div>"
        "<div class='card'><div class='label'>Oil Temp</div>"
        "<div class='value' id='oilT'>--</div><div class='unit'>&deg;C</div></div>"
        "<div class='card'><div class='label'>Water Temp</div>"
        "<div class='value' id='waterT'>--</div><div class='unit'>&deg;C</div></div>"
        "<div class='card'><div class='label'>Oil Pressure</div>"
        "<div class='value' id='oilP'>--</div><div class='unit'>Bar</div></div>"
        "<div class='card'><div class='label'>Throttle</div>"
        "<div class='value' id='thr'>--</div><div class='unit'>%</div></div>"
        "<div class='card'><div class='label'>Accel Pedal</div>"
        "<div class='value' id='accel'>--</div><div class='unit'>%</div></div>"
        "<div class='card'><div class='label'>Brake</div>"
        "<div class='value' id='brake'>--</div><div class='unit'>%</div></div>"
        "<div class='card'><div class='label'>Gauge Mode</div>"
        "<div class='value gauge-mode' id='gauge'>--</div></div>"
        "<div class='card'><div class='label'>Luminosity</div>"
        "<div class='value' id='lum'>--</div><div class='unit'>%</div></div>"
        "<div class='card'><div class='label'>Pot Voltage</div>"
        "<div class='value' id='potV'>--</div><div class='unit'>V</div></div>"
        "</div>"
        "<div class='status'>Uptime: <span id='up'>--</span>s</div>"
    );

    client.printf(
        "<script>"
        "function u(){fetch('/api/data').then(r=>r.json()).then(d=>{"
        "document.getElementById('rpm').textContent=d.engineRPM;"
        "document.getElementById('speed').textContent=d.speed.toFixed(1);"
        "document.getElementById('oilT').textContent=d.oilTemp.toFixed(1);"
        "document.getElementById('waterT').textContent=d.waterTemp.toFixed(1);"
        "document.getElementById('oilP').textContent=d.oilPressure.toFixed(2);"
        "document.getElementById('thr').textContent=d.throttlePos.toFixed(1);"
        "document.getElementById('accel').textContent=d.accelPos.toFixed(1);"
        "document.getElementById('brake').textContent=d.brakePercent;"
        "document.getElementById('gauge').textContent=d.gaugeType;"
        "document.getElementById('lum').textContent=d.luminosity;"
        "document.getElementById('potV').textContent=((d.luminosity-10)/90*3.3).toFixed(2);"
        "document.getElementById('up').textContent=d.uptime;"
        "}).catch(()=>{})}u();setInterval(u,%u);"
        "</script></body></html>",
        WebServerConfig::REFRESH_INTERVAL_MS
    );
}
