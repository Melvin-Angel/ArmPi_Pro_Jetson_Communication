# Pi startup launch snapshot — 6 November 2026

`start_functions.launch` is the actual file retrieved over SSH from
`/home/ubuntu/armpi_pro/src/armpi_pro_bringup/launch/start_functions.launch`
on the user's Pi. Both the Pi source and retrieved copy had SHA-256
`062fe86fec78365d6f6d1bf07411733131a28dc8dba5087ad03ce2b93be1e1bc`.

It documents how the Pi includes `wheel_odometry_startup.launch` in its existing
boot process. Its bridge binds to the Pi's then-current Wi-Fi address
`172.20.10.3`; this is environment-specific and can change. Compare the live
file and adapt network settings before applying it anywhere; do not overwrite
an existing robot launch without preserving its changes.
