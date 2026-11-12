# joycon-mouse

A Nintendo Joy-Con as a mouse on Linux. The gyroscope moves the cursor.
The buttons click. The stick scrolls.

This uses the mainline `hid-nintendo` driver. No extra kernel module.
`python-evdev` injects a virtual mouse through `/dev/uinput`. Buttons are
read from hidraw, because `joycond` keeps the button event node for root.

## Grip

Point the shoulder end at the screen. On a right Joy-Con that is the R/ZR
end. On a left Joy-Con it is the L/ZL end. Face the buttons toward the ceiling.

```
        R / ZR          point this end at the screen
      +--------+
      | stick  |
      |  Y  X  |
      |  B  A  |
      +--------+
         wrist          buttons face the ceiling
```

Twist left and right to move across. Tip that shoulder end up and down
to move vertically. The pointer measures rotation around gravity, so the
cursor still tracks if your wrist is rolled.

## Buttons

Right Joy-Con:

| Input | Action |
| --- | --- |
| ZR or A | Left click |
| R or B | Right click |
| Stick click | Middle click |
| Stick | Scroll |
| Hold X | Precision, about a third of the speed |
| + | Pause and resume |

Left Joy-Con uses ZL or Left, L or Down, stick click, Up for precision,
and − to pause.

A resting controller does not crawl. Startup averages about a second of
gyro bias, and anything slower than 1.2 degrees per second is ignored.

## Requirements

- Linux with `CONFIG_HID_NINTENDO` (mainline since 5.16)
- A paired Joy-Con, connected over Bluetooth
- Python 3 and `python-evdev`
- Write access to `/dev/uinput` (systemd-logind gives this to the seat user)
- `joycond` is fine to leave running

Pair a Joy-Con from the Bluetooth settings, or hold the sync button on
the rail and connect `Joy-Con (R)` or `Joy-Con (L)`.

## Install

```bash
git clone https://github.com/michaelheavener4/joycon-mouse
cd joycon-mouse
ln -sfn "$PWD/joycon_mouse.py" ~/.local/bin/joycon-mouse
mkdir -p ~/.config/systemd/user
cp joycon-mouse.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now joycon-mouse.service
```

The user service expects the clone at `~/joycon-mouse` only because that
is where the symlink points. The service itself runs `~/.local/bin/joycon-mouse`.

Watch it:

```bash
systemctl --user status joycon-mouse
journalctl --user -u joycon-mouse -f
```

## Tune

Cursor travel is `SENS` pixels per degree, near the top of `joycon_mouse.py`.
The default is 22, about 60 degrees of wrist to cross a 1366-wide screen.
`DEADZONE_DPS` is the stillness threshold.

## Tests

```bash
python3 -m unittest discover -s tests -v
```

The tests check the axis signs, the deadzone, and the button and stick
decoding. They do not need a Joy-Con.
