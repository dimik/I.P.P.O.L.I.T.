#!/usr/bin/env python3
"""Offline agent harness for the Radxa Q6A (Llama 3.2 1B on the NPU + MCP tools).

The 1B is a weak agent, so tool use is decomposed into steps a small model can
actually do, using the daemon's raw-prompt mode to control the Llama 3.2 chat
format and to *prefill* assistant turns:
  1. SELECT  - show a numbered tool menu, model replies with just a number
               (or 0 = answer now). Trivial classification, reliable.
  2. ARGS    - we inject the *valid* tool name and prefill the JSON so the model
               only fills arguments (no name hallucination). Args are filtered
               against the tool's schema.
  3. OBSERVE - dispatch via MCP, feed the result back, loop.

Two real MCP servers (web search + robot) are spawned as stdio subprocesses.

Usage:  python3 agent.py "your request"
"""
import asyncio
import json
import os
import re
import socket
import sys
from contextlib import AsyncExitStack

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

SOCK = os.environ.get("Q6A_LLM_SOCK", "/tmp/q6a-llm.sock")
HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
MAX_STEPS = 5
OBS_LIMIT = 1500
DEBUG = os.environ.get("AGENT_DEBUG", "1") == "1"

SERVERS = {
    "websearch": [PY, os.path.join(HERE, "mcp_websearch.py")],
    "robot": [PY, os.path.join(HERE, "mcp_robot.py")],
}


def log(*a):
    if DEBUG:
        print(*a, file=sys.stderr, flush=True)


# ---- local model call (raw mode over the daemon socket) ----
def query_llm(prompt):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect(SOCK)
    s.sendall(b"\x01RAW\x01" + prompt.encode("utf-8"))
    s.shutdown(socket.SHUT_WR)
    buf = b""
    while True:
        b = s.recv(4096)
        if not b:
            break
        buf += b
    return buf.decode("utf-8", "replace").strip()


def gen(history, extra_user, prefill=""):
    """Generate one assistant turn given history (ends after an <|eot_id|>),
    an injected user turn, and an optional assistant prefill."""
    prompt = (history
              + f"<|start_header_id|>user<|end_header_id|>\n\n{extra_user}<|eot_id|>"
              + "<|start_header_id|>assistant<|end_header_id|>\n\n" + prefill)
    return prefill + query_llm(prompt)


# ---- tolerant JSON extraction ----
def _json_objects(text):
    objs, i = [], 0
    while i < len(text):
        if text[i] == "{":
            depth, j = 0, i
            while j < len(text):
                if text[j] == "{":
                    depth += 1
                elif text[j] == "}":
                    depth -= 1
                    if depth == 0:
                        objs.append(text[i:j + 1])
                        break
                j += 1
            i = j + 1
        else:
            i += 1
    return objs


def render_result(res):
    parts = [getattr(c, "text", None) or str(c) for c in getattr(res, "content", []) or []]
    return ("\n".join(p for p in parts if p) or "(no output)")[:OBS_LIMIT]


async def main(user_msg):
    async with AsyncExitStack() as stack:
        tool_of, specs = {}, []          # name->session ; (name, desc, schema)
        for cmd in SERVERS.values():
            params = StdioServerParameters(command=cmd[0], args=cmd[1:])
            read, write = await stack.enter_async_context(stdio_client(params))
            sess = await stack.enter_async_context(ClientSession(read, write))
            await sess.initialize()
            for t in (await sess.list_tools()).tools:
                tool_of[t.name] = sess
                specs.append((t.name, (t.description or "").strip().splitlines()[0], t.inputSchema))
        log(f"[agent] tools: {', '.join(sorted(tool_of))}")

        menu = "\n".join(f"{i}. {n} — {d}" for i, (n, d, _) in enumerate(specs, 1))
        menu += "\n0. none — I can answer now"
        system = ("You are a robot assistant with tools. You have no reliable built-in "
                  "knowledge and cannot know the robot state without a tool.")
        history = ("<|begin_of_text|>"
                   f"<|start_header_id|>system<|end_header_id|>\n\n{system}<|eot_id|>"
                   f"<|start_header_id|>user<|end_header_id|>\n\n{user_msg}<|eot_id|>")

        for step in range(1, MAX_STEPS + 1):
            # 1. SELECT a tool by number (prefill a digit to force the format)
            sel = gen(history,
                      f"Which tool do you need next? Choose by number:\n{menu}\n"
                      "Reply with ONLY the number.", prefill="")
            m = re.search(r"\d+", sel)
            choice = int(m.group()) if m else 0
            log(f"[step {step}] select -> {choice}  (raw {sel!r})")
            if not (1 <= choice <= len(specs)):
                break                                   # 0 / invalid -> answer

            name, _, schema = specs[choice - 1]
            sprops = (schema or {}).get("properties", {})
            props = list(sprops)
            required = (schema or {}).get("required", [])
            str_keys = [p for p in props if sprops[p].get("type") == "string"]
            primary = next((k for k in required if sprops.get(k, {}).get("type") == "string"),
                           str_keys[0] if str_keys else None)
            # 2. ARGS
            args = {}
            if not props:
                pass                                            # no-arg tool
            elif primary:
                # prefill the primary string arg's opening quote, read the value directly
                raw = query_llm(history
                                + f"<|start_header_id|>user<|end_header_id|>\n\nCall {name} for: {user_msg}<|eot_id|>"
                                + "<|start_header_id|>assistant<|end_header_id|>\n\n"
                                + f'{{"name": "{name}", "parameters": {{"{primary}": "')
                val = raw.split('"')[0].split("\n")[0].strip()
                if val:
                    args = {primary: val}
            else:
                argp = gen(history, f"Call {name}. Arguments as JSON.",
                           prefill=f'{{"name": "{name}", "parameters": {{')
                for blob in _json_objects(argp):
                    try:
                        obj = json.loads(blob)
                    except Exception:
                        continue
                    if isinstance(obj, dict) and obj.get("name") == name:
                        args = {k: v for k, v in (obj.get("parameters") or {}).items() if k in props}
                        break
            log(f"[step {step}] {name}({args})")
            # 3. OBSERVE
            try:
                obs = render_result(await tool_of[name].call_tool(name, args))
            except Exception as e:                      # noqa: BLE001
                obs = f"tool error: {e}"
            log(f"[obs] {obs[:200]}")
            history += (f"<|start_header_id|>assistant<|end_header_id|>\n\n"
                        f'{{"name": "{name}", "parameters": {json.dumps(args)}}}<|eot_id|>'
                        f"<|start_header_id|>ipython<|end_header_id|>\n\n{obs}<|eot_id|>")
            if not re.match(r"(tool error|error executing)", obs, re.I):
                break                                   # got a useful result -> answer now

        # FINAL answer (plain text, no tools)
        print(gen(history,
                  "Using ONLY the tool results above, answer the user's question in plain text. "
                  "Extract the real answer from the content; do NOT list website titles, sources, "
                  "or URLs. If the results do not contain the answer, say you could not find it.",
                  prefill=""))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit('usage: python3 agent.py "your request"')
    asyncio.run(main(" ".join(sys.argv[1:])))
