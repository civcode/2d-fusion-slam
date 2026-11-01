"""Opt-in real-browser regression; requires Playwright and its Chromium browser.

Run against the demo with its synthetic map/replay:
  python tests/web/check_browser_interactions.py http://127.0.0.1:8765
Not part of unittest discovery: the normal suite needs only the standard library.
"""
from __future__ import annotations

import argparse

from playwright.sync_api import sync_playwright


def check_view(browser, base: str, replay: bool):
    page = browser.new_page(viewport={"width": 1600, "height": 1200})
    errors = []
    page.on("pageerror", lambda error: errors.append(str(error)))
    page.goto(base + ("/replay" if replay else "/"))
    state = "world" if replay else "view"
    stage = "replay-stage" if replay else "map-stage"
    page.wait_for_function("() => session !== null && frame !== null" if replay
                           else "() => mapImage !== null && metadata !== null")
    if replay:
        page.select_option("#overlay-map", "sample-synthetic-workshop.slammap")
        page.wait_for_function("() => mapImage !== null")
    page.locator("#" + stage).scroll_into_view_if_needed()
    box = page.locator("#" + stage).bounding_box()
    assert box
    x, y = box["x"] + box["width"] / 2, box["y"] + box["height"] / 2
    page.mouse.move(x, y)
    snapshot = lambda: page.evaluate(f"({{...{state}}})")

    # Horizontal/zero-delta wheel events must not be interpreted as zoom-out.
    before = snapshot()
    page.dispatch_event("#" + stage, "wheel", {"deltaX": 80, "deltaY": 0})
    assert snapshot() == before, "horizontal wheel changed zoom/pan"
    page.dispatch_event("#" + stage, "wheel", {"deltaY": 0})
    assert snapshot() == before, "zero wheel changed zoom/pan"
    page.dispatch_event("#" + stage, "wheel", {"deltaY": 100, "buttons": 4})
    assert snapshot() == before, "middle-button wheel changed zoom without a drag"

    # Observe cancellation after the application's target handlers have run.
    page.evaluate("""() => {
      window.middleDefaults = [];
      for (const type of ['pointerdown', 'mousedown', 'auxclick']) {
        document.addEventListener(type, event => {
          if (event.button === 1) middleDefaults.push([type, event.defaultPrevented]);
        });
      }
    }""")
    page.mouse.down(button="middle")
    page.wait_for_timeout(250)
    assert snapshot() == before, "holding middle button changed the view"
    # Ignore wheel input during a pan, including devices that emit wheel deltas
    # while the middle button is held. Test both event.buttons and drag state.
    for buttons in (0, 4):
        page.dispatch_event("#" + stage, "wheel", {"deltaY": 100, "buttons": buttons})
    assert snapshot() == before, "wheel during middle pan changed zoom"
    page.mouse.move(x + 40, y + 25, steps=5)
    after = snapshot()
    assert after["scale"] == before["scale"], "middle drag changed zoom"
    assert abs(after["x"] - before["x"] - 40) < 1
    assert abs(after["y"] - before["y"] - 25) < 1
    page.mouse.up(button="middle")
    assert snapshot() == after
    assert page.evaluate("drag === null")
    defaults = page.evaluate("middleDefaults")
    assert ["pointerdown", True] in defaults, defaults
    assert all(canceled for _, canceled in defaults), defaults
    if not replay:
        assert page.evaluate("chosenCell === null"), "middle click inspected a cell"
    page.mouse.move(x + 45, y + 30)
    assert snapshot() == after, "pan continued after release"
    page.mouse.click(x, y, button="middle")
    assert snapshot() == after, "middle click changed the view"
    if not replay:
        assert page.evaluate("chosenCell === null"), "middle click inspected a cell"

    # Wheel zoom still works and keeps the point under the cursor fixed.
    before = snapshot()
    page.dispatch_event("#" + stage, "wheel", {
        "deltaY": -100, "clientX": x, "clientY": y,
    })
    zoomed = snapshot()
    assert zoomed["scale"] > before["scale"]
    # MouseEvent client coordinates are integer CSS pixels.
    for axis, pixel in (("x", int(x) - box["x"]), ("y", int(y) - box["y"])):
        assert abs((pixel - before[axis]) / before["scale"] -
                   (pixel - zoomed[axis]) / zoomed["scale"]) < 1e-6
    page.dispatch_event("#" + stage, "wheel", {"deltaY": 100, "clientX": x, "clientY": y})
    assert abs(snapshot()["scale"] - before["scale"]) < 1e-9
    page.mouse.move(x, y)
    scale = snapshot()["scale"]
    page.mouse.wheel(0, -100)
    page.wait_for_function(f"() => {state}.scale > {scale}")

    # Right-button motion is not a pan. Existing left-button drag stays usable.
    page.mouse.move(x, y)
    before = snapshot()
    page.mouse.down(button="right")
    page.mouse.move(x + 20, y + 10)
    page.mouse.up(button="right")
    assert snapshot() == before, "right button panned the view"
    page.keyboard.press("Escape")
    page.mouse.move(x, y)
    page.mouse.down(button="left")
    page.mouse.move(x + 30, y + 20, steps=3)
    page.mouse.up(button="left")
    assert snapshot()["scale"] == before["scale"]
    assert abs(snapshot()["x"] - before["x"] - 30) < 1

    if not replay:
        page.mouse.click(x, y)
        page.wait_for_function("() => chosenCell !== null")
        page.locator("#cell-details").wait_for(state="visible")

    # Losing capture/canceling must not leave a stuck drag.
    for event in ("pointercancel", "lostpointercapture"):
        page.mouse.down(button="middle")
        page.dispatch_event("#" + stage, event, {
            "pointerId": page.evaluate("drag.pointerId"),
        })
        assert page.evaluate("drag === null"), event
        page.mouse.up(button="middle")
    assert not errors, errors
    print(("Replay" if replay else "Map") + ": image decode, pan, hold, wheel zoom and drag cleanup passed")
    page.close()


def check_fusion(browser, base, run):
    page = browser.new_page(viewport={"width": 1600, "height": 1200})
    errors = []
    page.on("pageerror", lambda error: errors.append(str(error)))
    report = page.request.get(base + "/api/fusion/runs/" + run).json()
    frames = report["frames"]
    page.goto(base + "/fusion")
    page.locator('#fusion-runs option[value="' + run + '"]').wait_for(state="attached")
    page.select_option("#fusion-runs", run)
    page.wait_for_function('n => !document.querySelector("#fusion-play").disabled && '
                           'document.querySelector("#fusion-position").textContent.includes(" / " + n)', arg=frames)
    page.wait_for_function('() => document.querySelector("#fusion-prof-fps").textContent !== "—"')
    page.select_option("#fusion-map", "fused_scan_matched")
    page.wait_for_function('() => document.querySelector("#fusion-map-note").textContent.includes("FINAL") || '
                           '!document.querySelector("#fusion-alert").hidden')
    assert page.locator("#fusion-alert").is_hidden(), page.locator("#fusion-alert").inner_text()
    page.click("#fusion-next")
    assert page.locator("#fusion-position").inner_text() == f"2 / {frames}"
    page.click("#fusion-open-map")
    page.wait_for_function('() => mapImage !== null || !document.querySelector("#warning").hidden')
    assert page.locator("#warning").is_hidden(), page.locator("#warning").inner_text()
    assert page.evaluate('() => selected') == "fusion@" + run + "@fused_scan_matched"
    assert page.evaluate('() => mapImage !== null')
    assert page.evaluate('() => metadata.source_id') == report["scene"]
    assert not errors, errors
    print(run + ": profile panel, replay step, final overlay and Map Explorer deep link passed")
    page.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("url", nargs="?", default="http://127.0.0.1:8765")
    parser.add_argument("--fusion-run", action="append", default=[],
                        help="Also verify this generated fusion run and its Map Explorer link")
    args = parser.parse_args()
    base = args.url.rstrip("/")
    with sync_playwright() as playwright:
        with playwright.chromium.launch() as browser:
            for replay in (False, True):
                check_view(browser, base, replay)
            for run in args.fusion_run:
                check_fusion(browser, base, run)
