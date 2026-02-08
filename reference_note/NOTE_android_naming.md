# Android Naming Notes (Quick Reference)

## 1. What is `com.example.phonebot_app_android`?

This is your **Android package name / application ID**.

Think of it as: - A unique identifier for your app - Similar to a
namespace - Required for app installation and Play Store publishing

Example format:

    com.companyname.appname

Examples: - Gmail → `com.google.android.gm` - WhatsApp →
`com.whatsapp` - Example template → `com.example.myapp`

`com.example` is just a placeholder automatically created by Android
Studio.

------------------------------------------------------------------------

## 2. Is it the real app name?

YES.

Even if it looks like an example, it is: - Your real package ID - Used
internally by Android - Used when publishing

Changing it later can be annoying, so better choose a good name early.

------------------------------------------------------------------------

## 3. Recommended naming style

Good options:

    edu.ucla.romela.phonebot
    com.yourname.phonebot
    com.labname.robotapp

Tips: - Use your organization/domain if possible - Keep it lowercase -
Avoid spaces/special characters

------------------------------------------------------------------------

## 4. Can you rename it later?

Yes, but carefully.

Never just rename folders manually. Use Android Studio refactor tools
instead.

If needed later: - Refactor package name - Update Gradle applicationId

------------------------------------------------------------------------

## 5. Useful Android Studio shortcuts (VSCode equivalents)

### Reopen recent files:

    Ctrl + E

### Open file by name:

    Ctrl + N

### Search everything:

    Double Shift

### Navigate back/forward in code:

    Ctrl + Alt + Left / Right

Closest to VSCode Ctrl+Shift+T workflow: - Ctrl+E (recent files) -
Double Shift (search everything)

------------------------------------------------------------------------

## 6. Mental model summary

Android app identity:

    Package Name = App Unique ID
    Not just example text
    Used everywhere

------------------------------------------------------------------------

(Generated for future reference)
