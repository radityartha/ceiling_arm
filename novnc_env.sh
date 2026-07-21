#!/usr/bin/env bash
#
# Source this file to point GUI apps (rqt, rviz, etc.) at the noVNC display.
#   usage:  source ./novnc_env.sh
#
# noVNC's web port is separate from the X11 display number.
# The X server the noVNC installer set up runs on display :1
# (VNC port 5901 = 5900 + 1); noVNC on port 22380 just proxies to it.
export NOVNC_PORT=22380
export DISPLAY=:1
export QT_X11_NO_MITSHM=1
# :1 is Xvnc (TigerVNC's virtual framebuffer X server), not a display bound
# to the host GPU, so there is no NVIDIA GLX driver to select here even
# though this host has one (Quadro P2200) — always use software GL.
export LIBGL_ALWAYS_SOFTWARE=1
export LP_NUM_THREADS=1
export LIBGL_DRI3_DISABLE=1

echo "[env ready] DISPLAY=$DISPLAY (noVNC port: $NOVNC_PORT, QT_X11_NO_MITSHM=1, LIBGL_ALWAYS_SOFTWARE=1)"
