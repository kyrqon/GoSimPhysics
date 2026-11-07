"""Offline browser smoke tests; run after `go generate ./examples/browser`.

Optional test dependency: `python3 -m pip install playwright` followed by
`python3 -m playwright install chromium`. A system Chromium is used when found.
Viewing the demo itself does not require Python or Playwright.
"""

import json
import os
from pathlib import Path
import shutil
import unittest
from urllib.parse import urlsplit, urlunsplit

from playwright.sync_api import sync_playwright


DEMO = Path(__file__).with_name("demo.html").resolve()
DEMO_URL = os.environ.get("GOSIM_DEMO_URL", DEMO.as_uri())
if DEMO_URL.startswith(("http:", "https:")):
    parts = urlsplit(DEMO_URL)
    DEMO_URL = urlunsplit(parts._replace(path=parts.path or "/"))


class BrowserDemoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not DEMO.is_file():
            raise RuntimeError("Generate demo.html with go generate ./examples/browser first")
        cls.playwright = sync_playwright().start()
        executable = os.environ.get("GOSIM_CHROMIUM") or shutil.which("chromium") or shutil.which("google-chrome")
        options = {"headless": True}
        if executable:
            options["executable_path"] = executable
        cls.browser = cls.playwright.chromium.launch(**options)

    @classmethod
    def tearDownClass(cls):
        cls.browser.close()
        cls.playwright.stop()

    def setUp(self):
        # Some managed browsers block file://. For those, allow just the initial
        # self-contained HTTP document, then go offline before every interaction.
        from_file = DEMO_URL.startswith("file:")
        self.context = self.browser.new_context(viewport={"width": 1440, "height": 1000}, offline=from_file)
        self.addCleanup(self.context.close)
        self.page = self.context.new_page()
        self.errors = []
        self.remote_requests = []
        self.page.on("pageerror", lambda error: self.errors.append(str(error)))
        self.page.on("request", lambda request: self.remote_requests.append(request.url) if request.url.startswith(("http:", "https:")) and request.url != DEMO_URL else None)
        self.page.goto(DEMO_URL, wait_until="load")
        self.page.wait_for_function("window.gosimDemo && window.demoState", timeout=30000)
        self.context.set_offline(True)

    def tearDown(self):
        self.assertEqual(self.errors, [], "browser JavaScript/Go runtime error")
        self.assertEqual(self.remote_requests, [], "offline demo requested network resources")

    def state(self):
        return self.page.evaluate("window.gosimDemo.getState()")

    def pause(self):
        self.page.evaluate("window.gosimDemo.pause()")

    def test_offline_engine_advances_and_handles_bad_input(self):
        self.page.wait_for_function("window.demoState.time > 0.1")
        self.assertEqual(self.state()["dimension"], 3)
        self.assertGreater(self.state()["bodyCount"], 8)
        self.pause()
        before = self.state()["time"]
        reply = json.loads(self.page.evaluate("window.gosim.step(-1)"))
        self.assertIn("error", reply)
        actual = json.loads(self.page.evaluate("window.gosim.state()"))
        self.assertEqual(actual["time"], before)
        self.assertTrue(self.page.locator("#fatal-overlay").is_hidden())

    def test_pause_step_spawn_impulse_and_settings(self):
        self.page.get_by_role("button", name="Pause simulation", exact=True).click()
        before = self.state()
        self.page.wait_for_timeout(150)
        self.assertEqual(self.state()["time"], before["time"])
        self.page.get_by_role("button", name="Single step", exact=True).click()
        self.assertAlmostEqual(self.state()["time"] - before["time"], 1 / 60, places=10)
        self.page.locator("#sphere-button").click()
        self.page.locator("#box-button").click()
        self.assertEqual(self.state()["bodyCount"], before["bodyCount"] + 2)
        energy = self.state()["energy"]
        self.page.locator("#kick-button").click()
        self.assertNotEqual(self.state()["energy"], energy)
        self.page.locator("#gravity-input").evaluate("e => { e.value = '5'; e.dispatchEvent(new Event('input', {bubbles:true})); }")
        self.page.locator("#substeps-input").select_option("8")
        self.assertEqual(self.state()["gravity"], 5)
        self.assertEqual(self.state()["substeps"], 8)
        self.page.get_by_role("button", name="Reset scene", exact=True).click()
        self.assertEqual(self.state()["time"], 0)
        self.assertEqual(self.state()["bodyCount"], before["bodyCount"])

    def test_scene_switch_and_real_3d_pendulum(self):
        self.pause()
        for name, dimension in [("rain", 3), ("pendulum", 3), ("billiards2d", 2), ("stack", 3)]:
            self.page.locator(f'[data-scene="{name}"]').click()
            self.assertEqual(self.state()["scene"], name)
            self.assertEqual(self.state()["dimension"], dimension)
            self.page.get_by_role("button", name="Single step", exact=True).click()
            self.assertGreater(self.state()["time"], 0)
            if name == "pendulum":
                self.assertLess(self.state()["jointError"], 0.01)
                bob = next(body for body in self.state()["bodies"] if not body["static"])
                self.assertGreater(abs(bob["position"]["z"]), 0)
            if dimension == 2:
                self.page.get_by_role("button", name="Circle", exact=True).click()
                self.assertEqual(self.state()["bodies"][-1]["shape"], "circle")

    def test_camera_and_mobile_layout(self):
        self.pause()
        canvas = self.page.locator("#simulation-canvas")
        self.page.wait_for_timeout(80)
        before = canvas.screenshot()
        bounds = canvas.bounding_box()
        x, y = bounds["x"] + bounds["width"] * 0.6, bounds["y"] + bounds["height"] * 0.4
        self.page.mouse.move(x, y)
        self.page.mouse.down()
        self.page.mouse.move(x + 100, y + 35, steps=8)
        self.page.mouse.up()
        self.page.wait_for_timeout(80)
        self.assertNotEqual(before, canvas.screenshot(), "camera drag did not redraw the view")
        self.page.set_viewport_size({"width": 390, "height": 844})
        self.page.wait_for_timeout(100)
        self.assertTrue(self.page.evaluate("document.documentElement.scrollWidth <= window.innerWidth + 1"), "mobile layout overflows horizontally")
        self.assertGreater(canvas.bounding_box()["height"], 240)


if __name__ == "__main__":
    unittest.main(verbosity=2)
