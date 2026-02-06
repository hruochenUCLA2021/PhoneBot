# NOTE — Manually Add App Link to Ubuntu Dock (Linux .desktop Launcher)

This note explains how to **manually add an application (like Android Studio) to Ubuntu's left dock / app launcher** when the built-in "Create Desktop Entry" option is missing.

This method works for:
- Android Studio
- JetBrains IDEs
- AppImages
- Custom tools
- Portable Linux apps

---

## 📌 Why This Works

Ubuntu (GNOME) uses **`.desktop` launcher files** to:
- Show apps in the application menu
- Allow pinning to the left dock
- Display proper icons
- Group windows correctly

We will create one manually.

---

## 📁 Step 1 — Create Applications Folder (If Needed)

```bash
mkdir -p ~/.local/share/applications
```

---

## 📝 Step 2 — Create Desktop File

```bash
nano ~/.local/share/applications/android-studio.desktop
```

---

## 📄 Step 3 — Paste This Configuration

```ini
[Desktop Entry]
Version=1.0
Type=Application
Name=Android Studio
Comment=Android Development IDE
Exec=/opt/android-studio/bin/studio
Icon=/opt/android-studio/bin/studio.png
Categories=Development;IDE;
Terminal=false
StartupWMClass=jetbrains-studio
```

---

## 🔧 Step 4 — Make It Executable

```bash
chmod +x ~/.local/share/applications/android-studio.desktop
```

---

## 🔄 Step 5 — Refresh App Database

```bash
update-desktop-database ~/.local/share/applications
```

---

## 📌 Step 6 — Pin to Ubuntu Dock

1. Press **Super (Windows Key)**
2. Search: `Android Studio`
3. Right-click → **Add to Favorites**

Now it will stay in the left sidebar permanently.

---

## 🔍 Troubleshooting

### If It Doesn't Show Up:
Log out and log back in, or run:
```bash
gtk-launch android-studio
```

If that launches the app, GNOME recognizes the launcher.

---

## 🏠 If App Is Installed in Home Instead of /opt

Edit this line in the `.desktop` file:
```ini
Exec=/opt/android-studio/bin/studio
```
Change it to something like:
```ini
Exec=/home/YOUR_USERNAME/apps/android-studio/bin/studio
```

---

## 🧠 Pro Tips

### Works for Any App
Just change:
- `Name=` → App name
- `Exec=` → Path to executable
- `Icon=` → Path to icon

### Find App Icon
Common locations:
```bash
/opt/<app>/bin/
/usr/share/icons/
```

---

## ✅ Result

You now have:
- One-click launch from dock
- Proper icon
- Window grouping
- Terminal-free startup

---

## 🏁 Tested On
- Ubuntu 20.04
- Ubuntu 22.04
- Ubuntu 24.04
