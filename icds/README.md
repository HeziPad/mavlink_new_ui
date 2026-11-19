# Payload ICDs

Each `*.json` file here describes one payload's MAVLink ICD: how its commands ride in `COMMAND_LONG`, and how its
reports ride in a carrier message with the payload's own byte layout. The GCS loads every file at start-up and,
with no new code:
- decodes the payload's reports;
- shows them live in the **PAYLOAD** panel (the chip next to HUD on the video);
- sends any of its commands with the ICD's ACK policy.

The same file drives the payload simulators (e.g. `nextvision_mavlink_sim`), so the GCS and the simulator always
agree on byte layouts.

Search order:
1. `$GCS_ICD_DIR`
2. `<app dir>/icds`
3. `<prefix>/share/mavlink_gcs_ui/icds` (Linux install)
4. the app bundle's `Resources/icds` (macOS)
5. the source tree (build directory)

## Adding a payload

1. Write `icds/<id>.json` (schema below). Its reports and commands then appear in the PAYLOAD panel.
2. Optionally, for deeper integration (its line of sight on the map and video, tap-to-track, quick actions), write
   `src/payload/adapters/<id>.cpp` implementing `payload::PayloadAdapter` (`src/payload/adapter.hpp`). Then add
   one line to `makeAdapter()` in `src/payload/adapter.cpp`. `adapters/nextvision.cpp` is the example.
3. Add byte fixtures to `tests/test_payload.cpp` if you have reference frames.

A file that fails to load is reported as an error event at start-up (bad JSON, unknown field type, or a declared
length that disagrees with the fields). Other ICDs still load.

## Schema 1

```jsonc
{
  "schema": 1,
  "id": "nextvision_trip",              // file id; adapters are keyed by it
  "name": "NextVision TRIP",
  "icd_version": "…",
  "match": { "sysid": 1, "compid": 100 },   // whose reports / ACKs these are (omit a key = any)
  "annotations": ["free text: decisions about ICD inconsistencies"],

  "enums":     { "SystemModeReport": { "0": "Stow", "1": "Pilot", … } },   // value labels
  "bitfields": { "BitStatus": { "0": "InternalComms", "15": "CameraCommsError" } },

  "reports": {
    "carrier": { "msg": "V2_EXTENSION", "msg_id": 248, "id_field": "message_type", "id_bytes": 2 },
    "types": [
      { "id": 0, "name": "System", "default_hz": 10, "length": 78,   // length: optional check, incl. id bytes
        "fields": [
          { "name": "roll", "type": "f32", "unit": "deg" },
          { "name": "system_mode", "type": "u8", "enum": "SystemModeReport" },
          { "name": "recording_status", "type": "u8", "nibbles": "RecordingState" },  // per-channel nibbles
          { "name": "bit_status", "type": "u16", "bits": "BitStatus" },
          { "name": "lat", "type": "f32", "invalid": 400.0 }                         // shown as "--"
        ]},
      { "id": 9, "name": "ObjectDetection", "fields": [ … header … ],
        "records": { "name": "detections", "size": 32, "max": 7, "fields": [ … ] } }  // repeated records
    ]
  },

  "commands": {
    "carrier": { "msg": "COMMAND_LONG", "command": 203, "target_system": 1, "target_component": 0,
                 "selector": "param1", "sub_selector": "param2" },
    "ack": { "timeout_ms": 2000, "retries": 3 },
    "list": [
      { "name": "SetSystemMode", "id": 0, "params": [ { "name": "mode", "enum": "SystemModeCmd" }, … ] },
      { "name": "Illuminator.Safety", "id": 19, "sub": 4, "periodic_hz": 10, "params": [ … ] },
      { "name": "SetGimbal", "id": 6, "ack": "none", "stream_hz": 20, "params": [ … ] },
      { "name": "Config.Get", "id": 31, "sub": 3, "ack": "report:15", "params": [ … ] },
      { "name": "SetROI", "command": 201, "params": [ …7 params… ] }      // a standard MAV_CMD
    ]
  },
  "config_params": [ { "id": 28, "name": "ReportSystemID", "min": 0, "max": 255 }, … ]
}
```

### Reports

- **Byte layout:** fields are packed little-endian, in order, after the report-type bytes (`id_bytes`).
- **Field types:** `u8 i8 u16 i16 u32 i32 u64 i64 f32 f64 char[N] u8[N]`.
- **Variable-length reports:** a fixed header, then up to `max` records of `size` bytes.
- **Decoding is tolerant:**
  - A short payload reads as zeros (MAVLink 2 truncates trailing zero bytes).
  - Records are counted rounding up.
  - Extra trailing bytes from newer firmware are ignored.

### Commands

- **Encoding:** `param1` = `id`, `param2` = `sub` (when given). The command's own params follow, in order, up to
  `param7`. Without an `id`, the params fill param1…param7 of the given standard `command`.
- **`ack` policies:**
  - `"ack"` (default): wait for COMMAND_ACK, retry after `timeout_ms`, up to `retries` times.
  - `"none"`: fire and forget.
  - `"report:N"`: answered by report type N.
- **Queue:** payload commands go out one at a time, through a queue separate from the vehicle's commands, so a
  camera command never pre-empts a flight action.
- **Parameter `enum` values:** an enum name, `@config_params` or `@reports`. The panel steps through enum values
  by their labels.
