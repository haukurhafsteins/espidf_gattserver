# GATT Server for the espidf Framework

## Overview
What I want to accomplish is to have a single gattserver component that I can use in different applications and abstracts the most common code used when dealing with gatt:
- Have the possibility to register/create one or more services.
- Have the possibility to register a character to a given service.
- If a registered character is writable, have the possibility to register a callback that will be called when a client writes to the character.
- Have the possibility to trigger a notification for a character

This GATT server provides a **generic** and **modular** BLE implementation for ESP32.
Users can define their **own UUIDs** and dynamically register **float, int, and string characteristics**. The system also supports **notifications** and **write callbacks**.

## Features
- **Dynamic Service UUID:** Set the primary service UUID at initialization.
- **Custom Characteristics:** Register characteristics dynamically using user-defined UUIDs.
- **Notification Support:** Send real-time updates to BLE clients.
- **Client Write Support:** Register callbacks for client write events.

---

## 1️⃣ Define UUIDs in a Header File
Before using the GATT server, users **must create a UUID header file** (e.g., `my_uuid_config.h`). This allows them to **easily modify UUIDs** without changing the GATT server code.

**Example `my_uuid_config.h`:**
```c
#pragma once

// Define the Primary Service UUID
#define MY_SERVICE_UUID "00001234-0000-1000-8000-00805f9b34fb"

// Define Characteristic UUIDs
#define MY_CHAR_UUID_TEMPERATURE "00005678-0000-1000-8000-00805f9b34fb"
#define MY_CHAR_UUID_HUMIDITY "00005679-0000-1000-8000-00805f9b34fb"
#define MY_CHAR_UUID_PRESSURE "0000567A-0000-1000-8000-00805f9b34fb"
```

---

## 2️⃣ Register Characteristics
### 📌 Register a Float Characteristic
```c
gatt_param_handle_t temp_char = gattserver_register_float("Temperature", MY_CHAR_UUID_TEMPERATURE,
                                    ESP_GATT_PERM_READ, ESP_GATT_CHAR_PROP_BIT_NOTIFY, 22.5);
```

### 📌 Register an Integer Characteristic
```c
gatt_param_handle_t pressure_char = gattserver_register_int("Pressure", MY_CHAR_UUID_PRESSURE,
                                     ESP_GATT_PERM_READ, ESP_GATT_CHAR_PROP_BIT_READ, 1013);
```

### 📌 Register a String Characteristic
```c
gatt_param_handle_t humidity_char = gattserver_register_string("Humidity", MY_CHAR_UUID_HUMIDITY,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_WRITE, "50%");
```

---

## 3️⃣ Initialize the GATT Server
Call `gattserver_init()` with **your service name and UUID**:
```c
#include "gattserver.h"
#include "my_uuid_config.h"

gattserver_init("My BLE Device", MY_SERVICE_UUID);
```
The server will register all characteristics previously registered in 2️⃣.
---

## 4️⃣ Send Notifications
The ESP32 can **send updates** to connected clients when the values change.

### 📌 Notify a Float Value Change
```c
gattserver_notify_float(temp_char, 23.1);
```

### 📌 Notify an Integer Value Change
```c
gattserver_notify_int(pressure_char, 1020);
```

### 📌 Notify a String Value Change
```c
gattserver_notify_string(humidity_char, "55%");
```

---

## 5️⃣ Handle Client Writes
To allow **clients to modify a characteristic**, register a write callback.

### 📌 Register a Write Callback
```c
esp_err_t status = gattserver_register_write_cb(humidity_char, evloop->loop_handle, evloop->base,
    [](gatt_param_handle_t param, void *data, size_t len) {
        printf("Humidity updated: %s\n", (char *)data);
    });
```
🔹 This callback is called **when a client writes** to the **Humidity** characteristic.

---

## ✅ Summary
| **Feature**      | **Function**                              | **Example** |
|-----------------|--------------------------------|-----------|
| **Initialize**  | `gattserver_init(name, uuid)` | Set service UUID |
| **Register Float**  | `gattserver_register_float()` | Temperature (°C) |
| **Register Int**    | `gattserver_register_int()`   | Pressure (hPa) |
| **Register String** | `gattserver_register_string()` | Humidity (%) |
| **Notify Float**    | `gattserver_notify_float()`   | Update temperature |
| **Notify Int**      | `gattserver_notify_int()`     | Update pressure |
| **Notify String**   | `gattserver_notify_string()`  | Update humidity |
| **Write Callback**  | `gattserver_register_write_cb()` | Handle client writes |

---

