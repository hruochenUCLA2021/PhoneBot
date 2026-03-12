# Android USB Motor Issue Note

## What happened

The Android app found the USB motor-related device, tried to request USB permission, and then failed before enabling the motor hardware.

The root cause shown on the screen is an **Android 14 / API 34 PendingIntent restriction**.

---

## Exact words shown on the screenshot

```text
dxlg (latest)
Motor controller idle.

1773235380183ms: Motors OFF (torque disable)...
1773235380193ms: Motor controller idle.
1773235388504ms: Searching USB serial device...
1773235388514ms: USB drivers found: 1
1773235388515ms: First device:
UsbDevice(name=/dev/bus/usb/002/002,
vid=0x403, pid=0x6014)
1773235388516ms: Requesting USB
permission for UsbDevice(name=/dev/bus/usb/
002/002, vid=0x403, pid=0x6014) ...
1773235388521ms: Enable failed:
IllegalArgumentException:
com.example.phonebot_app_android:
Targeting U+ (version 34 and above) disallows
creating or retrieving a PendingIntent with
FLAG_MUTABLE, an implicit Intent within and
without FLAG_NO_CREATE and
FLAG_ALLOW_UNSAFE_IMPLICIT_INTENT for
security reasons. To retrieve an already
existing PendingIntent, use FLAG_NO_CREATE,
however, to create a new PendingIntent with
an implicit Intent use FLAG_IMMUTABLE.
1773235388872ms: Motors OFF (torque
disable)...
1773235388877ms: Motor controller idle.

Motor HW: OFF (tap to ON)

Motor Feedback (XL430 present state)
```

---

## What the problem is

Your app is trying to request **USB permission** using a `PendingIntent` pattern that is no longer allowed when targeting **Android U+ / version 34 and above**.

The key error is:

```text
IllegalArgumentException:
Targeting U+ (version 34 and above) disallows creating or retrieving a PendingIntent with FLAG_MUTABLE ...
```

So the sequence is:

1. The app successfully finds the USB device.
2. The app tries to request permission for that device.
3. Android rejects the `PendingIntent`.
4. The enable step fails.
5. Motor hardware stays off.

---

## Why this happens

On newer Android versions, especially **Android 14 (API 34)**, security rules around `PendingIntent` are stricter.

The app appears to be using a `PendingIntent` with:

- `FLAG_MUTABLE`
- an **implicit Intent**

That combination is blocked for this use case.

---

## What to change

### Preferred fix

When creating the `PendingIntent` for the USB permission flow, use:

- an **explicit Intent** if possible
- `FLAG_IMMUTABLE` instead of `FLAG_MUTABLE`

### If retrieving an existing PendingIntent

Use:

- `FLAG_NO_CREATE`

But for creating a new one with this flow, the message on screen explicitly says to use:

- `FLAG_IMMUTABLE`

---

## Typical code pattern to update

### Old pattern that can fail on Android 14

```java
Intent intent = new Intent(ACTION_USB_PERMISSION);
PendingIntent permissionIntent =
    PendingIntent.getBroadcast(
        context,
        0,
        intent,
        PendingIntent.FLAG_MUTABLE
    );
```

### Safer pattern

```java
Intent intent = new Intent(context, UsbPermissionReceiver.class);
intent.setAction(ACTION_USB_PERMISSION);

PendingIntent permissionIntent =
    PendingIntent.getBroadcast(
        context,
        0,
        intent,
        PendingIntent.FLAG_IMMUTABLE
    );
```

---

## Important idea: implicit vs explicit intent

### Implicit intent
This does **not** name the target receiver class directly.

Example:

```java
new Intent(ACTION_USB_PERMISSION)
```

### Explicit intent
This directly names the receiver class.

Example:

```java
new Intent(context, UsbPermissionReceiver.class)
```

For Android 14 compatibility, using an **explicit broadcast receiver intent** together with `FLAG_IMMUTABLE` is usually the safer fix.

---

## Example USB permission flow

```java
public static final String ACTION_USB_PERMISSION =
    "com.example.phonebot_app_android.USB_PERMISSION";

IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
context.registerReceiver(usbReceiver, filter, Context.RECEIVER_NOT_EXPORTED);

Intent intent = new Intent(context, UsbPermissionReceiver.class);
intent.setAction(ACTION_USB_PERMISSION);

PendingIntent permissionIntent = PendingIntent.getBroadcast(
    context,
    0,
    intent,
    PendingIntent.FLAG_IMMUTABLE
);

usbManager.requestPermission(device, permissionIntent);
```

---

## What this means for your motor app

The app is **not failing to find the USB device**.

It already found:

```text
UsbDevice(name=/dev/bus/usb/002/002, vid=0x403, pid=0x6014)
```

So the main issue is **not USB discovery**.

The issue is specifically the **permission request mechanism** used before enabling the motor controller.

---

## Symptoms you can expect

- Device appears in logs
- USB driver count is nonzero
- Permission request starts
- App throws `IllegalArgumentException`
- Motor stays off
- Button still shows:

```text
Motor HW: OFF (tap to ON)
```

---

## How to solve it

### Fix checklist

- Find where `usbManager.requestPermission(...)` is called
- Find where the corresponding `PendingIntent.getBroadcast(...)` is created
- Replace `FLAG_MUTABLE` with `FLAG_IMMUTABLE` when creating a new permission intent
- Make the intent **explicit** by targeting a receiver class
- Check the broadcast receiver registration and action string
- Rebuild and test again on Android 14 / API 34+

---

## Quick summary

**Problem:**  
The app uses a USB permission `PendingIntent` that Android 14 blocks.

**Why:**  
`FLAG_MUTABLE` + implicit intent is rejected for this case on target SDK 34+.

**Fix:**  
Use an **explicit intent** and create the `PendingIntent` with **`FLAG_IMMUTABLE`**.

---

## Personal reminder

When I see logs like:

```text
Targeting U+ (version 34 and above) disallows creating or retrieving a PendingIntent with FLAG_MUTABLE...
```

I should immediately check:

1. `PendingIntent.getBroadcast(...)`
2. whether the intent is implicit or explicit
3. whether I should switch to `FLAG_IMMUTABLE`

