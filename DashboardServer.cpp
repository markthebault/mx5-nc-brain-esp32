#include "DashboardServer.h"
#include "Config.h"
#include "Debug.h"
#include <WiFi.h>
#include <NetworkClient.h>
#include <WiFiAP.h>
#include <Update.h>

// --- Module State ---
static NetworkServer server(WebServerConfig::PORT);
static TelemetryData* pData = nullptr;

// --- Forward Declarations ---
static void serverTask(void* pvParameters);
static void handleClient(NetworkClient& client);

struct HttpRequest {
    String method;  // "GET" or "POST"
    String path;
    String contentType;
    int contentLength;
};

static HttpRequest parseRequest(NetworkClient& client);
static void sendJsonResponse(NetworkClient& client);
static void sendHtmlResponse(NetworkClient& client);
static void sendOtaPage(NetworkClient& client);
static void handleOtaUpload(NetworkClient& client, HttpRequest& req);

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
    HttpRequest req = parseRequest(client);

    if (req.method == "GET" && req.path == "/api/data") {
        sendJsonResponse(client);
    } else if (req.method == "GET" && req.path == "/update") {
        sendOtaPage(client);
    } else if (req.method == "POST" && req.path == "/update") {
        handleOtaUpload(client, req);
    } else {
        sendHtmlResponse(client);
    }
}

static HttpRequest parseRequest(NetworkClient& client) {
    HttpRequest req;
    req.method = "GET";
    req.path = "/";
    req.contentLength = 0;

    unsigned long timeout = millis() + 2000;

    while (client.connected() && millis() < timeout) {
        if (client.available()) {
            String line = client.readStringUntil('\n');
            line.trim();

            // Parse request line: "GET /path HTTP/1.1" or "POST /path HTTP/1.1"
            if (line.startsWith("GET ") || line.startsWith("POST ")) {
                int methodEnd = line.indexOf(' ');
                req.method = line.substring(0, methodEnd);
                int pathEnd = line.indexOf(' ', methodEnd + 1);
                if (pathEnd > methodEnd + 1) {
                    req.path = line.substring(methodEnd + 1, pathEnd);
                }
            }

            if (line.startsWith("Content-Type:")) {
                req.contentType = line.substring(13);
                req.contentType.trim();
            }

            if (line.startsWith("Content-Length:")) {
                req.contentLength = line.substring(15).toInt();
            }

            // Empty line = end of headers
            if (line.length() == 0) break;
        }
    }
    return req;
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
        "<div class='status'>Uptime: <span id='up'>--</span>s | <a href='/update' style='color:#0ff'>Firmware Update</a></div>"
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

static void sendOtaPage(NetworkClient& client) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();

    client.print(
        "<!DOCTYPE html><html><head>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>MX5 NC - Firmware Update</title>"
        "<style>"
        "*{box-sizing:border-box;margin:0;padding:0}"
        "body{font-family:system-ui,sans-serif;background:#1a1a2e;color:#e0e0e0;"
        "padding:16px;display:flex;flex-direction:column;align-items:center}"
        "h1{color:#0ff;font-size:1.4em;margin-bottom:20px}"
        ".box{background:#16213e;border-radius:10px;padding:24px;text-align:center;"
        "border:1px solid #0f3460;max-width:400px;width:100%}"
        "input[type=file]{margin:16px 0;color:#e0e0e0}"
        "input[type=submit]{background:#0f3460;color:#0ff;border:1px solid #0ff;"
        "padding:10px 24px;border-radius:6px;font-size:1em;cursor:pointer}"
        "input[type=submit]:hover{background:#0ff;color:#1a1a2e}"
        "#prog{margin-top:16px;display:none}"
        ".back{margin-top:16px;color:#0ff;font-size:.85em}"
        "</style></head><body>"
        "<h1>Firmware Update</h1>"
        "<div class='box'>"
        "<form method='POST' action='/update' enctype='multipart/form-data' id='fm'>"
        "<p>Select .bin firmware file:</p>"
        "<input type='file' name='firmware' accept='.bin' required><br>"
        "<input type='submit' value='Upload & Flash'>"
        "</form>"
        "<div id='prog'><p id='msg'>Uploading... do not disconnect.</p></div>"
        "</div>"
        "<div class='back'><a href='/' style='color:#0ff'>&larr; Back to Dashboard</a></div>"
        "<script>"
        "document.getElementById('fm').onsubmit=function(){"
        "document.getElementById('prog').style.display='block';"
        "};"
        "</script>"
        "</body></html>"
    );
}

static void handleOtaUpload(NetworkClient& client, HttpRequest& req) {
    bool success = false;

    // Extract multipart boundary from Content-Type
    int bIdx = req.contentType.indexOf("boundary=");
    if (bIdx < 0 || req.contentLength <= 0) {
        client.println("HTTP/1.1 400 Bad Request");
        client.println("Connection: close");
        client.println();
        client.print("Missing boundary or content length");
        return;
    }
    String boundary = "--" + req.contentType.substring(bIdx + 9);

    // Skip multipart part headers (read until empty line after boundary)
    unsigned long timeout = millis() + 5000;
    bool headersEnded = false;
    while (client.connected() && millis() < timeout) {
        if (client.available()) {
            String line = client.readStringUntil('\n');
            line.trim();
            if (line.length() == 0) {
                headersEnded = true;
                break;
            }
        }
    }

    if (!headersEnded) {
        client.println("HTTP/1.1 400 Bad Request");
        client.println("Connection: close");
        client.println();
        client.print("Failed to parse upload headers");
        return;
    }

    // Begin OTA update
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        DEBUG_PRINTF("OTA begin failed: %s\n", Update.errorString());
        client.println("HTTP/1.1 500 Internal Server Error");
        client.println("Connection: close");
        client.println();
        client.print("Update.begin() failed");
        return;
    }

    DEBUG_PRINTLN("OTA upload started...");

    // Read binary data in chunks, stop at closing boundary
    uint8_t buf[512];
    size_t totalWritten = 0;
    timeout = millis() + 60000; // 60s timeout for upload

    while (client.connected() && millis() < timeout) {
        int avail = client.available();
        if (avail <= 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        int toRead = (avail < (int)sizeof(buf)) ? avail : (int)sizeof(buf);
        int bytesRead = client.read(buf, toRead);
        if (bytesRead <= 0) break;

        // Check if we hit the closing boundary
        // The boundary appears as \r\n--boundary-- at the end
        // We write everything and trim the boundary after
        size_t written = Update.write(buf, bytesRead);
        if (written != (size_t)bytesRead) {
            DEBUG_PRINTF("OTA write error: %s\n", Update.errorString());
            break;
        }
        totalWritten += written;
    }

    if (Update.end(true)) { // true = set final size
        DEBUG_PRINTF("OTA success! %u bytes written\n", totalWritten);
        success = true;
    } else {
        DEBUG_PRINTF("OTA end failed: %s\n", Update.errorString());
    }

    // Send response
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();

    if (success) {
        client.print(
            "<!DOCTYPE html><html><head>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<style>body{font-family:system-ui,sans-serif;background:#1a1a2e;color:#0f8;padding:40px;text-align:center}</style>"
            "</head><body>"
            "<h1>Firmware Updated Successfully!</h1>"
            "<p>Rebooting in 3 seconds...</p>"
            "</body></html>"
        );
        client.stop();
        delay(3000);
        ESP.restart();
    } else {
        client.print(
            "<!DOCTYPE html><html><head>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<style>body{font-family:system-ui,sans-serif;background:#1a1a2e;color:#f44;padding:40px;text-align:center}</style>"
            "</head><body>"
            "<h1>Firmware Update Failed</h1>"
            "<p><a href='/update' style='color:#0ff'>Try again</a></p>"
            "</body></html>"
        );
    }
}
