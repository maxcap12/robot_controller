# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import omni.timeline
import omni.ui as ui
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.api.world import World
from isaacsim.core.utils.prims import get_prim_object_type
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.gui.components.element_wrappers import CollapsableFrame, DropDown, FloatField, TextBlock, StateButton, CheckBox
from isaacsim.examples.extension.core_connectors import LoadButton, ResetButton
from isaacsim.gui.components.ui_utils import get_style
from omni.usd import StageEventType

from .robot_controller import RobotController
from .map_creator import load_map

class UIBuilder:
    def __init__(self):
        self.robot_name = "spot"
        self.robot = None
        self.robot_controller = RobotController()
        self.use_sgraphs = False
        self.custom_map = False

        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []

        # UI elements created using a UIElementWrapper from isaacsim.gui.components.element_wrappers
        self.wrapped_ui_elements = []

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        # Run initialization for the provided example
        self._on_init()

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """Callback for when the UI is opened from the toolbar.
        This is called directly after build_ui().
        """
        # Reset internal state when UI window is closed and reopened
        # self._invalidate_articulation()

        self._selection_menu.repopulate()

        # Handles the case where the user loads their Articulation and
        # presses play before opening this extension
        # if self._timeline.is_playing():
        #     self._stop_text.visible = False
        # elif self._timeline.is_stopped():
        #     self._stop_text.visible = True

    def on_timeline_event(self, event):
        """Callback for Timeline events (Play, Pause, Stop)

        Args:
            event (omni.timeline.TimelineEventType): Event Type
        """
        pass

    def on_physics_step(self, step):
        """Callback for Physics Step.
        Physics steps only occur when the timeline is playing

        Args:
            step (float): Size of physics step
        """
        pass

    def on_stage_event(self, event):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """
        if event.type == int(omni.usd.StageEventType.ASSETS_LOADED):  # Any asset added or removed
            self._selection_menu.repopulate()
        elif event.type == int(StageEventType.OPENED):
            self._reset_extension()
        elif event.type == int(omni.usd.StageEventType.SIMULATION_START_PLAY):  # Timeline played
            # Treat a playing timeline as a trigger for selecting an Articulation
            self._selection_menu.trigger_on_selection_fn_with_current_selection()
        elif event.type == int(omni.usd.StageEventType.SIMULATION_STOP_PLAY):  # Timeline stopped
            # Ignore pause events
            if self._timeline.is_stopped():
                pass

    def cleanup(self):
        """
        Called when the stage is closed or the extension is hot reloaded.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from isaacsim.gui.components.element_wrappers implement a cleanup function that should be called
        """
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    def build_ui(self):
        """
        Build a custom UI tool to run your extension.
        This function will be called any time the UI window is closed and reopened.
        """
        selection_panel_frame = CollapsableFrame("Selection Panel")

        with selection_panel_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._selection_menu = DropDown(
                    "Select Robot",
                    tooltip="Select desired robot",
                    on_selection_fn=self._on_robot_selection,
                    keep_old_selections=True,
                    populate_fn = lambda: ["spot", "go1", "go2"]
                )
                # This sets the populate_fn to find all USD objects of a certain type on the stage, overriding the populate_fn arg
                # Figure out the type of an object with get_prim_object_type(prim_path)
                # self._selection_menu.set_populate_fn_to_find_all_usd_objects_of_type("articulation", repopulate=False)

                self._sgraphs_checkbox = CheckBox(
                    "Show SGraphs",
                    default_value=False,
                    on_click_fn=self._update_use_sgraphs
                )
                self.wrapped_ui_elements.append(self._scenario_state_btn)
                
                self._select_map = CheckBox(
                    "Select Custom Map",
                    default_value=False,
                    on_click_fn=self._load_custom_map
                )
                
                self._load_btn = LoadButton(
                    "Add Button", "ADD", setup_scene_fn=self._setup_robot
                )
                self._load_btn.set_world_settings(physics_dt=1 / 60.0, rendering_dt=1 / 60.0)
                self.wrapped_ui_elements.append(self._load_btn)

                self._reset_btn = ResetButton(
                    "Reset Button", "RESET", pre_reset_fn=None, post_reset_fn=self._on_post_reset_btn
                )
                self._reset_btn.enabled = False
                self.wrapped_ui_elements.append(self._reset_btn)

                self._scenario_state_btn = StateButton(
                    "Run Scenario",
                    "RUN",
                    "STOP",
                    on_a_click_fn=self._on_run_scenario_a_text,
                    on_b_click_fn=self._on_run_scenario_b_text,
                    physics_callback_fn=self._update_scenario,
                )
                self._scenario_state_btn.enabled = False

                
    ######################################################################################
    # Functions Below This Point Support The Provided Example And Can Be Replaced/Deleted
    ######################################################################################

    def _on_init(self):
        pass

    def _on_robot_selection(self, selection: str):
        self.robot_name = selection

    def _setup_robot(self):
        self.robot = self.robot_controller.load_robot(self.robot_name, self.use_sgraphs)
        world = World.instance()
        world.scene.add(self.robot)

        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True
        self._reset_btn.enabled = True
    
    def _on_post_reset_btn(self):
        self.robot_controller.reset()
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True

    def _on_run_scenario_a_text(self):
        self._timeline.play()
        self.robot_controller.play()

    def _on_run_scenario_b_text(self):
        self._timeline.pause()
        self.robot_controller.pause()

    def _update_scenario(self, step):
        self.robot_controller.update(step)

    def _update_use_sgraphs(self, use_sgraphs):
        self.use_sgraphs = use_sgraphs
        
    def _load_custom_map(self, map_path):
        self.custom_map = map_path
        load_map(map_path)