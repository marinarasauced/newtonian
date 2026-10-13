
from __future__ import annotations

from typing import TYPE_CHECKING

import newton

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


_PANEL_MARGIN = 10
_PANEL_ALPHA = 0.7
_SKY_COLOR = (38.0 / 255.0, 36.0 / 255.0, 38.0 / 255.0)


class NewtonianViewer:
    """
    """

    def __init__(self, sim: NewtonianSimulator) -> None:
        """
        """
        self.gl = newton.viewer.ViewerGL()
        self._render_setup()
        self._ui_setup()
        self.attach(sim=sim)

    def attach(self, sim: NewtonianSimulator) -> NewtonianViewer:
        """
        """
        self.sim = sim
        self.gl.set_model(model=sim.model)
        self._render_planes()
        # register custom ui elements upon model changes
        if self.gl.gui is not None:
            self.gl.register_ui_callback(callback=self._ui_control, position="free")
        return self

    @property
    def is_paused(self) -> bool:
        """
        """
        return self.gl._paused

    @is_paused.setter
    def is_paused(self, is_paused: bool) -> None:
        """
        """
        self.gl._paused = is_paused

    def is_running(self) -> bool:
        """
        """
        return self.gl.is_running()

    def stepable(self) -> bool:
        """
        """
        return self.gl.should_step()

    def render(self, time: float, state: newton.State, contacts: newton.Contacts | None) -> None:
        """
        Renders one frame for the simulator's Newton GL viewer.
        """
        self.gl.begin_frame(time=time)
        self.gl.log_state(state=state)
        if contacts is not None:
            self.gl.log_contacts(contacts=contacts, state=state)
        self.gl.end_frame()

    def close(self) -> None:
        """
        Closes the simulator's Newton GL viewer.
        """
        self.gl.close()

    def apply(self, state: newton.State) -> None:
        """
        Applies forces in the simulator's Newton GL viewer.
        """
        self.gl.apply_forces(state=state)

    def _render_setup(self) -> None:
        """
        Sets the sky to one flat color.
        """
        renderer = self.gl.renderer
        renderer.sky_upper = _SKY_COLOR
        renderer.sky_lower = _SKY_COLOR

    def _render_planes(self) -> None:
        """
        Removes the checker pattern from planes.
        """
        for batch in self.gl._shape_instances.values():
            if batch.geo_type == newton.GeoType.PLANE:
                materials = batch.materials.numpy()
                materials[:, 2] = 0.0
                batch.materials.assign(materials)

    def _ui_setup(self) -> None:
        """
        Sets up custom ui elements.
        """
        viewer = self.gl
        viewer.wind = None
        if viewer.gui is None:
            return
        self._ui_stats = (0.0, 0.0)
        viewer.gui._render_left_panel = lambda: None    # type: ignore[method-assign]
        viewer.register_ui_callback(callback=self._ui_measure, position="stats")

    def _ui_measure(self, imgui) -> None:
        """
        Measures and writes ui position and size to stats attribute.
        """
        pos, size = imgui.get_window_pos(), imgui.get_window_size()
        self._ui_stats = (pos.y + size.y, size.x)

    def _ui_control(self, imgui) -> None:
        """
        Custom ui elements for simulator control.
        """
        viewer = self.gl
        gui = viewer.gui
        if gui is None:
            return

        scale = gui.ui.dpi_scale
        bottom, width = self._ui_stats

        # button size
        size = imgui.ImVec2(width - 2 * imgui.get_style().window_padding.x, 0)

        # button placement
        imgui.set_next_window_pos(imgui.ImVec2(imgui.get_io().display_size.x - _PANEL_MARGIN * scale, bottom + _PANEL_MARGIN * scale), pivot=imgui.ImVec2(1.0, 0.0))
        imgui.set_next_window_bg_alpha(_PANEL_ALPHA)

        # overlay flags
        flags = (imgui.WindowFlags_.no_decoration.value | imgui.WindowFlags_.always_auto_resize.value | imgui.WindowFlags_.no_saved_settings.value | imgui.WindowFlags_.no_focus_on_appearing.value | imgui.WindowFlags_.no_nav.value | imgui.WindowFlags_.no_move.value)

        # overlay
        expanded, _ = imgui.begin("Simulation Controls", flags=flags)
        if expanded:

            # pause
            if imgui.button("Resume" if self.is_paused else "Pause", size):
                self.sim.pause(is_paused=not self.is_paused)

            # step
            imgui.begin_disabled(not self.is_paused)
            if imgui.button("Step", size):
                viewer._step_requested = True
            imgui.end_disabled()

            # reset
            if imgui.button("Reset", size):
                self.sim.reset()

        imgui.end()
