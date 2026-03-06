# Joystick Controls

## Driver Controller — PS5 (Port 0)

| Input | Trigger | Function |
|---|---|---|
| Left Stick Y | continuous | Drive forward / backward |
| Left Stick X | continuous | Strafe left / right |
| Right Stick X | continuous | Rotate chassis |
| Square | while held | Face point — rotate to face fixed field coordinate (11.945, 4.029) |
| Circle | toggle | Toggle intake roller run / stop |
| Triangle | on press | Zero gyro heading (reset field-relative forward) |
| Cross (X) | on press | Toggle extend / retract arm (runs motor for 1.5 s then stops) |

---

## Operator Controller — PS5 (Port 1)

*(No bindings assigned)*

---

## Notes
- Driving is field-relative with slew rate limiting enabled.
- Extend/retract timeout (`kExtendTimeoutSeconds`) is set in `BallIntakeConstants` in `Constants.java`.
