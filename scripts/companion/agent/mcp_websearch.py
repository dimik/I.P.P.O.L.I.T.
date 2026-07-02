#!/usr/bin/env python3
"""MCP server: keyless web search via DuckDuckGo. One tool: web_search.

Runs as a stdio MCP server (spawned by the agent harness). The offline local
Llama calls this when it needs current/external information.
"""
from mcp.server.fastmcp import FastMCP
from ddgs import DDGS

mcp = FastMCP("websearch")


def _search(query: str, max_results: int = 5) -> str:
    results = []
    with DDGS() as ddgs:
        for i, r in enumerate(ddgs.text(query, max_results=max_results), 1):
            title = r.get("title", "")
            body = r.get("body", "")
            href = r.get("href", "") or r.get("url", "")
            results.append(f"{i}. {title}\n   {body}\n   {href}")
    return "\n".join(results) if results else "No results found."


@mcp.tool()
def web_search(query: str, max_results: int = 5) -> str:
    """Search the web for current information, facts, news, or anything you are
    unsure about. Returns the top results as title / snippet / url.

    Args:
        query: what to search for
        max_results: how many results to return (default 5)
    """
    try:
        return _search(query, max_results)
    except Exception as e:  # noqa: BLE001 - surface to the agent as an observation
        return f"web_search error: {e}"


if __name__ == "__main__":
    mcp.run()  # stdio transport
