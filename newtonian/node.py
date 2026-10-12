
from __future__ import annotations

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from std_srvs.srv import SetBool, Trigger

from newtonian.modules.params import Parameters
from newtonian.modules.physics.simulator import NewtonianSimulator


_ROS_PARAMETERS = ("use_sim_time", "start_type_description_service")


def _qos(item) -> QoSProfile:
    """
    """
    if item.qos == "sensor_data":
        return qos_profile_sensor_data
    if item.qos == "default":
        return QoSProfile(depth=item.qos_depth)
    raise ValueError(f"'{item.name}' must be 'default' or 'sensor_data'")


class NewtonianNode(Node):
    """
    """
    timer = None

    def __init__(self) -> None:
        """
        """
        super().__init__("newtonian", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.sim: NewtonianSimulator | None = None
        self.publishers_: dict[str, Publisher] = {}
        self.subscriptions_: list[Subscription] = []
        self.clock_pub = self.create_publisher(msg_type=Clock, topic="/clock", qos_profile=10)
        self.create_service(srv_type=Trigger, srv_name="~/reset", callback=self._on_reset)
        self.create_service(srv_type=Trigger, srv_name="~/rebuild", callback=self._on_rebuild)
        self.create_service(srv_type=SetBool, srv_name="~/pause", callback=self._on_pause)
        self._rebuild()

    def _params(self) -> Parameters:
        """
        """
        names = self.list_parameters([], 0).names
        flat = {n: p.value for n, p in zip(names, self.get_parameters(names)) if n not in _ROS_PARAMETERS}
        return Parameters(flat)

    def _teardown(self) -> None:
        """
        """
        if self.timer is not None:
            self.destroy_timer(self.timer)
        for subscription in self.subscriptions_:
            self.destroy_subscription(subscription)
        for publisher in self.publishers_.values():
            self.destroy_publisher(publisher)
        self.subscriptions_.clear()
        self.publishers_.clear()
        if self.sim is not None:
            self.sim.close()
            self.sim = None

    def _rebuild(self) -> None:
        """
        """
        p = self._params()
        old = self.sim
        viewer = old.viewer if old is not None else None
        try:
            sim = NewtonianSimulator(p=p, viewer=viewer)
        except Exception:
            if old is not None and viewer is not None and viewer.sim is not old:
                viewer.attach(sim=old)
            raise
        if old is not None and sim.viewer is viewer:
            old.viewer = None
        self._teardown()
        self.sim = sim

        for s in self.sim.sensors:
            self.publishers_[s.name] = self.create_publisher(s.msg_type, s.topic, _qos(s))

        for i in self.sim.inputs:
            self.subscriptions_.append(self.create_subscription(i.msg_type, i.topic, i.callback, _qos(i)))

        rtf = p.get_float(key="sim.real_time_factor", default=1.0, above=0.0)
        period = self.sim.frame_dt / rtf
        self.timer = self.create_timer(timer_period_sec=period, callback=self._on_timer)

    def _stamp(self, sim: NewtonianSimulator) -> Time:
        """
        """
        ns = round(sim.frame * 1.0e+9 / sim.frame_rate)
        return Time(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)

    def _on_timer(self) -> None:
        """
        """
        sim = self.sim
        if sim is None or not self.context.ok():
            return
        if sim.viewer is not None and not sim.viewer.is_running():
            self.get_logger().info("viewer closed, shutting down ...")
            rclpy.try_shutdown()
            return
        if sim.stepable():
            sim.step()
            stamp = self._stamp(sim=sim)
            self.clock_pub.publish(Clock(clock=stamp))
            for s in sim.sense():
                self.publishers_[s.name].publish(s.to_msg(stamp=stamp))
        sim.render()

    def _on_reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        """
        if self.sim is None:
            response.success = False
            response.message = "no scene"
            return response
        self.sim.reset()
        response.success = True
        return response

    def _on_rebuild(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        """
        try:
            self._rebuild()
            response.success = True
        except Exception as e:
            self.get_logger().error("rebuild failed")
            response.success = False
            response.message = str(e)
        return response

    def _on_pause(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """
        """
        if self.sim is not None:
            self.sim.pause(is_paused=bool(request.data))
            response.message = "paused" if self.sim.is_paused else "running"
        response.success = self.sim is not None
        return response

    def destroy_node(self) -> None:
        """
        """
        self._teardown()
        super().destroy_node()


def main(args=None) -> None:
    """
    """
    rclpy.init(args=args)
    node = NewtonianNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
