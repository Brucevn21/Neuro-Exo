# NSF TCP Control Protocol (Client-Facing Guide)

## 1) Scope
This document describes the TCP protocol exposed by `tcp_server_task_nsf` in `main/rebless_wifi.c`.
It is written only for TCP clients (PC/app side).

---

## 2) Connection

- **Protocol:** TCP
- **Address:** `<device-ip>`
- **Port:** `11999`
- **Server behavior:** accepts a client and processes short text commands.

---

## 3) Request/Response Format

## 3.1 Request format

- Request is text.
- Fields are separated by **tab** (`\t`).
- Parsed command length must be `< 30` bytes.

Format:

```text
<mode>[\t<value>]
```

Examples:

```text
0
1\t0.75
2\t45.0
6\t1.20
```

## 3.2 Response format

For each parsed request, server returns one line:

```text
<drive_mode>\t<actual_curr>\t<actual_pos>\t<max_curr>\n
```

Example:

```text
0\t0.000\t0.000\t0.000
```

---

## 4) Command Table

| Mode | Meaning | Value required |
|---|---|---:|
| `0` | Status request | No |
| `1` | Set desired current | Yes |
| `2` | Set desired position | Yes |
| `3` | Set measure mode | No |
| `4` | Set current-control mode | No |
| `5` | Set position-control mode | No |
| `6` | Set max current | Yes |
| `7` | Start (same meaning as Motor ON) | No |
| `8` | Stop (same meaning as Motor OFF) | No |
| `9` | Reserved / no effective action | No |

---

## 5) Required Operation Rules

## 5.1 Mode `0` can be used anytime

- `0` is a safe status poll command.
- It is valid regardless of current control mode or run state.

## 5.2 Measure mode must be set before running

- Before actual device operation, **measure mode setup is required**.
- Reason: operation is constrained to the ROM domain, so startup must pass through measure mode.
- Recommended first step:

```text
3
```

## 5.3 Start/Stop semantics for `7`/`8`

- `7` means **Start**.
- `8` means **Stop**.
- Typical valid start transitions:
  - `3 -> 7`
  - `4 -> 7`
  - `5 -> 7`

---

## 6) Value Rules (for client input)

- Mode `1` current input is clamped to `[-3.0, 3.0]`.
- Mode `2` position input is clamped to `[-150.0, 150.0]`.
- Mode `6` max current input is clamped to `[-3.0, 3.0]`.

If input is out of range, server applies an internal clamp and still replies with a status line.

---

## 7) Recommended Client Scenarios

## Scenario A: Basic startup for ROM-area operation

1. Connect to `tcp://<device-ip>:11999`
2. Send `3` (measure mode)
3. Optionally send `0` to check status
4. Send `7` (start)
5. During run, poll with `0`
6. Send `8` (stop)

## Scenario B: Current-control start

1. Send `4` (current-control mode)
2. Send `1\t<current>`
3. Send `7` (start)
4. Poll with `0`
5. Send `8` (stop)

## Scenario C: Position-control start

1. Send `5` (position-control mode)
2. Send `2\t<position>`
3. Send `7` (start)
4. Poll with `0`
5. Send `8` (stop)

## Scenario D: Connection loss handling

1. If TCP disconnects, reconnect from client side.
2. After reconnect, re-send required mode and setpoint commands.
3. Use `0` to confirm current status before re-starting.

---

## 8) Practical Client Notes

- Keep each request short (`<30` bytes).
- Send one command per request.
- Use tab (`\t`) for commands requiring a value (`1`, `2`, `6`).
- Read one response line after each command.
- Use retry/reconnect logic for unstable networks.

---

## 9) Security Note

- This interface is plaintext TCP without built-in authentication.
- Use only in trusted/internal networks.
