import pytest


def pytest_collection_modifyitems(config, items):
    if not any("fusion" in item.keywords for item in items):
        return
    from tools.fusion_mcp import FusionClient

    if FusionClient(timeout=3).ping():
        return
    skip = pytest.mark.skip(reason="Fusion MCP server not reachable (set FUSION_MCP_URL)")
    for item in items:
        if "fusion" in item.keywords:
            item.add_marker(skip)
