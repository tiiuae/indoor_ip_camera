# TOPOTEK v41 Camera Setup Guide

This document explains how to use the **TOPOTEK v41** application to configure and access the camera on a **Linux system** using **Wine** and an **Ethernet connection**.

---

## Overview

The TOPOTEK v41 software is provided as a Windows executable (`.exe`).  
On Linux, it must be run using **Wine**.

The camera has a fixed IP address: **192.168.1.108**

To communicate with the camera, your computer must be on the same subnet: **192.168.1.xxx**


---

## 1. Install Wine

The TOPOTEK application requires Wine to run on Linux.

```bash
sudo apt update
sudo apt install -y wine64 wine32
wine --version
```
---

## 2. Run TOPOTEK App
```bash
cd path/to/topotek_app/
wine TOPOTEK_v41.exe
```

## 3. Connect to the Camera

1. Connect the camera to your computer using an **Ethernet cable**
2. Launch the **TOPOTEK v41** application using Wine
3. Connect to the camera from within the TOPOTEK v41 interface

---

## 4. Camera Access

Once the connection is established, you will be able to:

- Access the camera configuration panel
- Modify network and streaming settings
- Adjust image parameters (exposure, gain, resolution, etc.)
- Verify camera status and connectivity
