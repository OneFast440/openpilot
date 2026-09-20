"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

BigParamStepper's arithmetic and param round-trip.

Asset loading is stubbed: mici widgets pull textures and fonts through gui_app, which needs a
real window, so the rendering half cannot run here. The half that decides what gets written to
the param can, and that is the half a wrong tap would corrupt.
"""
import unittest
from unittest import mock

from openpilot.common.params import Params
from openpilot.system.ui.widgets import MousePos


class FakeTexture:
  width = 402
  height = 180


def make_stepper(param="FordLowSpeedFactor_ang", min_value=0.5, max_value=1.5, step=0.05):
  with mock.patch("openpilot.system.ui.lib.application.gui_app.texture", return_value=FakeTexture()), \
       mock.patch("openpilot.system.ui.lib.application.gui_app.font", return_value=None), \
       mock.patch("openpilot.selfdrive.ui.mici.widgets.button.gui_app.texture", return_value=FakeTexture()), \
       mock.patch("openpilot.selfdrive.ui.sunnypilot.mici.widgets.stepper.gui_app.font", return_value=None):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.stepper import BigParamStepper
    return BigParamStepper("test", param, min_value, max_value, step=step)


def tap(stepper, left: bool):
  x = stepper._rect.x + (10 if left else stepper._rect.width - 10)
  stepper._handle_mouse_release(MousePos(x, stepper._rect.y + 10))


class TestBigParamStepper(unittest.TestCase):
  # a real key: Params rejects names it does not know, and this is one the stepper drives
  PARAM = "FordLowSpeedFactor_ang"

  def setUp(self):
    self.params = Params()
    original = self.params.get(self.PARAM)
    self.params.remove(self.PARAM)

    def restore():
      if original is None:
        self.params.remove(self.PARAM)
      else:
        self.params.put(self.PARAM, float(original), block=True)

    self.addCleanup(restore)

  def test_starts_at_the_params_own_default_when_unset(self):
    """Not a default the widget carries: the param's registered one, so the two UIs and
    sunnylink all agree on what an untouched setting means."""
    s = make_stepper()
    self.assertEqual(s._value, 100)
    self.assertEqual(s.value, "1.00")

  def test_reads_back_what_the_large_screen_ui_stores(self):
    """Both UIs store the physical float, so a value set on one reads on the other."""
    self.params.put(self.PARAM, 1.15, block=True)
    s = make_stepper()
    self.assertEqual(s._value, 115)
    self.assertEqual(s.value, "1.15")

  def test_right_half_steps_up_and_left_half_steps_down(self):
    s = make_stepper()
    tap(s, left=False)
    self.assertEqual(s._value, 105)
    self.assertAlmostEqual(float(self.params.get(self.PARAM)), 1.05)
    tap(s, left=True)
    tap(s, left=True)
    self.assertEqual(s._value, 95)
    self.assertEqual(s.value, "0.95")

  def test_clamps_at_both_ends_without_writing(self):
    s = make_stepper(min_value=0.5, max_value=1.5)
    self.params.put(self.PARAM, 1.5, block=True)
    s.refresh()
    before = self.params.get(self.PARAM)
    tap(s, left=False)
    self.assertEqual(s._value, 150)
    self.assertEqual(self.params.get(self.PARAM), before)
    for _ in range(40):
      tap(s, left=True)
    self.assertEqual(s._value, 50)

  def test_a_value_outside_the_range_is_pulled_in(self):
    self.params.put(self.PARAM, 9.0, block=True)
    s = make_stepper(min_value=0.5, max_value=1.5)
    self.assertEqual(s._value, 150)

  def test_refresh_after_an_out_of_range_write(self):
    s = make_stepper()
    self.params.put(self.PARAM, 9.0, block=True)
    s.refresh()
    self.assertEqual(s._value, 150)

  def test_refresh_picks_up_an_external_change(self):
    s = make_stepper()
    self.params.put(self.PARAM, 1.30, block=True)
    s.refresh()
    self.assertEqual(s._value, 130)
    self.assertEqual(s.value, "1.30")

  def test_disabled_does_not_write(self):
    s = make_stepper()
    s.set_enabled(False)
    before = self.params.get(self.PARAM)
    tap(s, left=False)
    self.assertEqual(s._value, 100)
    self.assertEqual(self.params.get(self.PARAM), before)


if __name__ == "__main__":
  unittest.main()
