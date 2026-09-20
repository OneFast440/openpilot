"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

A numeric param control for the mici UI.

mici has a boolean toggle (BigParamControl) and a cycling multi-select (BigMultiParamToggle),
but nothing for a continuous value, so the vehicle tuning factors had no widget to live in.
Cycling twenty-one options with taps is not a control, so this splits the button instead: the
left half steps down, the right half steps up, and the value sits under the title.

Storage matches the large-screen OptionControlSP exactly: the param holds the physical float
and the widget works in hundredths internally, so a value set here reads back the same on the
large-screen UI and through sunnylink.
"""
import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.widgets import MousePos

SCALE = 100
GLYPH_SIZE = 56
GLYPH_PADDING = 30


class BigParamStepper(BigButton):
  def __init__(self, text: str, param: str, min_value: float, max_value: float, step: float = 0.05):
    super().__init__(text, "")
    self._param = param
    self._params = Params()
    self._min = round(min_value * SCALE)
    self._max = round(max_value * SCALE)
    self._step = max(1, round(step * SCALE))
    self._font = gui_app.font(FontWeight.BOLD)
    self.refresh()

  def _stored(self) -> int:
    """The param in hundredths. The param itself holds the physical value as a float, and an
    unset one falls back to its own registered default, exactly as OptionControlSP does."""
    raw = self._params.get(self._param, return_default=True)
    try:
      return round(float(raw) * SCALE)
    except (TypeError, ValueError):
      return self._min

  def refresh(self) -> None:
    self._value = min(self._max, max(self._min, self._stored()))
    self.set_value(f"{self._value / SCALE:.2f}")

  def _handle_mouse_release(self, mouse_pos: MousePos) -> None:
    super()._handle_mouse_release(mouse_pos)
    if not self.enabled:
      return
    direction = -1 if mouse_pos.x < self._rect.x + self._rect.width / 2 else 1
    new_value = min(self._max, max(self._min, self._value + direction * self._step))
    if new_value == self._value:
      self.trigger_shake()
      return
    self._value = new_value
    # blocking, as mici's own BigParamControl does: a tap then a quick page exit must not
    # lose the write
    self._params.put(self._param, self._value / SCALE, block=True)
    self.set_value(f"{self._value / SCALE:.2f}")

  def _subtitle_width_hint(self) -> int:
    # leave room for the two glyphs the value sits between
    return int(self._rect.width - (GLYPH_PADDING + GLYPH_SIZE) * 2)

  def _draw_content(self, btn_y: float) -> None:
    super()._draw_content(btn_y)
    alpha = 0.9 if self.enabled else 0.35
    color = rl.Color(255, 255, 255, int(255 * alpha))
    y = btn_y + self._rect.height - GLYPH_PADDING - GLYPH_SIZE
    at_min, at_max = self._value <= self._min, self._value >= self._max
    for glyph, x, greyed in (
      ("-", self._rect.x + GLYPH_PADDING, at_min),
      ("+", self._rect.x + self._rect.width - GLYPH_PADDING - GLYPH_SIZE, at_max),
    ):
      c = rl.Color(255, 255, 255, int(255 * 0.25)) if greyed else color
      rl.draw_text_ex(self._font, glyph, rl.Vector2(x, y), GLYPH_SIZE, 0, c)
