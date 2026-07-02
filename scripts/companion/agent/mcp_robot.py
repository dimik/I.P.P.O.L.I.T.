#!/usr/bin/env python3
"""MCP server: robot interaction via Valetudo's REST API (Dreame D10s Pro).

Runs as a stdio MCP server. Wraps the robot's HTTP API so the local Llama can
observe and (with care) control the robot. ROBOT_URL env overrides the target
(default the WiFi Valetudo host). Read-only/safe tools are always available;
movement tools return clear errors if the robot is unreachable.
"""
import os
import httpx
from mcp.server.fastmcp import FastMCP

ROBOT_URL = os.environ.get("ROBOT_URL", "http://192.168.1.213").rstrip("/")
mcp = FastMCP("robot")
_client = httpx.Client(base_url=ROBOT_URL, timeout=5.0)


def _get(path: str):
    r = _client.get(path)
    r.raise_for_status()
    return r.json()


def _put(path: str, body: dict) -> str:
    r = _client.put(path, json=body)
    r.raise_for_status()
    return f"ok ({r.status_code})"


@mcp.tool()
def robot_get_status() -> str:
    """Get the robot's current high-level status (e.g. docked, cleaning, error)."""
    try:
        attrs = _get("/api/v2/robot/state/attributes")
        status = [a for a in attrs if a.get("__class") == "StatusStateAttribute"]
        if status:
            s = status[0]
            return f"status={s.get('value')} flag={s.get('flag')}"
        return f"attributes: {attrs}"
    except Exception as e:  # noqa: BLE001
        return f"robot unreachable at {ROBOT_URL}: {e}"


@mcp.tool()
def robot_get_battery() -> str:
    """Get the robot's battery level (percent) and charging state."""
    try:
        attrs = _get("/api/v2/robot/state/attributes")
        bat = [a for a in attrs if a.get("__class") == "BatteryStateAttribute"]
        if bat:
            b = bat[0]
            return f"battery={b.get('level')}% flag={b.get('flag')}"
        return "no battery attribute reported"
    except Exception as e:  # noqa: BLE001
        return f"robot unreachable at {ROBOT_URL}: {e}"


@mcp.tool()
def robot_locate() -> str:
    """Make the robot beep / announce itself so you can find it. Safe: no movement."""
    try:
        return _put("/api/v2/robot/capabilities/LocateCapability", {"action": "locate"})
    except Exception as e:  # noqa: BLE001
        return f"locate failed ({ROBOT_URL}): {e}"


@mcp.tool()
def robot_dock() -> str:
    """Send the robot back to its charging dock."""
    try:
        return _put("/api/v2/robot/capabilities/BasicControlCapability", {"action": "home"})
    except Exception as e:  # noqa: BLE001
        return f"dock failed ({ROBOT_URL}): {e}"


@mcp.tool()
def robot_pause() -> str:
    """Pause whatever the robot is currently doing."""
    try:
        return _put("/api/v2/robot/capabilities/BasicControlCapability", {"action": "pause"})
    except Exception as e:  # noqa: BLE001
        return f"pause failed ({ROBOT_URL}): {e}"


@mcp.tool()
def robot_go_to(x: int, y: int) -> str:
    """Drive the robot to a point on its map. MOVEMENT: coordinates are Valetudo
    map units (mm). Only use after checking status; prefer robot_pause to stop."""
    try:
        return _put("/api/v2/robot/capabilities/GoToLocationCapability",
                    {"action": "goto", "coordinates": {"x": x, "y": y}})
    except Exception as e:  # noqa: BLE001
        return f"go_to failed ({ROBOT_URL}): {e}"


if __name__ == "__main__":
    mcp.run()
