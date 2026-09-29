# Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
# Advanced Remanufacturing and Technology Centre
# A*STAR Research Entities (Co. Registration No. 199702110H)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from uuid import uuid4

from res_mapf_planning.traffic_dependencies.models.plan import Plan, PlanId, Waypoint
from res_mapf_planning.traffic_dependencies.models.traffic_dependency import (
    TrafficDependency,
)
from res_plan_execution.plan_execution.dependency_manager import DependencyManager


def _make_plan() -> Plan:
    return Plan(
        plan_id=PlanId(destination_session=uuid4(), plan_version=0),
        waypoints=[
            Waypoint(name="A", position=(0.0, 0.0), progress=0.0),
            Waypoint(name="B", position=(1.0, 0.0), progress=1.0),
            Waypoint(name="C", position=(2.0, 0.0), progress=2.0),
        ],
    )


def test_compute_commit_cut_after_failed_plan():
    """
    on_plan_failed clears a robot's plan to None but keeps the robot in
    the dependency manager so other robots stay blocked on it until a
    replan arrives. compute_commit_cut must not crash while iterating
    that robot's own (now-None) plan.
    """
    dm = DependencyManager()
    dm.set_plan("robot_0", _make_plan())

    dm.on_plan_failed("robot_0")

    result = dm.compute_commit_cut()

    assert "robot_0" not in result.committed_locations, (
        "A robot with no plan cannot have a committed location"
    )


def test_compute_commit_cut_blocker_referencing_failed_plan():
    """
    A departure blocker can reference a robot whose plan has since failed:
    on_plan_failed clears that robot's plan to None without marking its
    plan_id complete, so other robots must remain blocked on it.
    Resolving that blocker must not crash on the None plan.
    """
    dm = DependencyManager()

    robot_a_plan = _make_plan()
    dm.set_plan("robot_a", robot_a_plan)

    blocker = TrafficDependency(
        name="robot_a",
        plan_id=robot_a_plan.plan_id,
        required_progress=1.0,
    )
    robot_b_plan = Plan(
        plan_id=PlanId(destination_session=uuid4(), plan_version=0),
        waypoints=[
            Waypoint(
                name="X", position=(0.0, 1.0), progress=0.0, departure_blockers=[blocker]
            ),
            Waypoint(name="Y", position=(1.0, 1.0), progress=1.0),
        ],
    )
    dm.set_plan("robot_b", robot_b_plan)

    dm.on_plan_failed("robot_a")

    dm.compute_commit_cut()
