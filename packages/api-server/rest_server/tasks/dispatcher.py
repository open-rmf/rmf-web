import rclpy
from fastapi.encoders import jsonable_encoder
from rclpy.node import Node
from rmf_task_msgs.msg import Delivery, Loop, TaskSummary, TaskType
from rmf_task_msgs.srv import CancelTask, GetTaskList, SubmitTask


class StatusModel:
    task_id: str
    state: str
    done: bool
    fleet_name: str
    robot_name: str
    task_type: str
    priority: int
    submited_start_time: int
    start_time: int
    end_time: int
    description: str
    progress: str


# dispatcher class
class DispatcherClient(Node):
    def __init__(self):
        super().__init__("dispatcher_client")
        self.submit_task_srv = self.create_client(SubmitTask, "/submit_task")
        self.get_tasks_srv = self.create_client(GetTaskList, "/get_tasks")
        self.cancel_task_srv = self.create_client(CancelTask, "/cancel_task")

        # detect if rmf is up
        while not self.submit_task_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Dispatcher node is not avail, waiting...")

    def submit_task_request(self, req_msg) -> str:
        """
        Task Submission - This function will trigger a ros srv call to the
        dispatcher node, and return a response. Function will return a Task ID
        """
        try:
            future = self.submit_task_srv.call_async(req_msg)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            response = future.result()
            if response and response.success:
                return response.task_id
        except Exception as e:
            self.get_logger().error("Error! Submit Srv failed %r" % (e,))
        return ""

    def convert_task_request(self, task_json):
        """
        :param (obj) task_json:
        :return req_msgs, error_msg
        This is to convert a json task req format to a rmf_task_msgs
        task_profile format. add this accordingly when a new msg field
        is introduced.
        The 'start time' here is refered to the "Duration" from now.
        """
        req_msg = SubmitTask.Request()

        try:
            if (
                ("task_type" not in task_json)
                or ("start_time" not in task_json)
                or ("description" not in task_json)
            ):
                raise Exception("Key value is incomplete")

            if "priority" in task_json and task_json["priority"] is not None:
                priority = int(task_json["priority"])
                if priority < 0:
                    raise Exception("Priority value is less than 0")
                req_msg.description.priority.value = priority
            else:
                req_msg.description.priority.value = 0

            desc = task_json["description"]
            if task_json["task_type"] == "Clean":
                req_msg.description.task_type.type = TaskType.TYPE_CLEAN
                req_msg.description.clean.start_waypoint = desc["cleaning_zone"]
            elif task_json["task_type"] == "Loop":
                req_msg.description.task_type.type = TaskType.TYPE_LOOP
                loop = Loop()
                loop.num_loops = int(desc["num_loops"])
                loop.start_name = desc["start_name"]
                loop.finish_name = desc["finish_name"]
                req_msg.description.loop = loop
            elif task_json["task_type"] == "Delivery":
                req_msg.description.task_type.type = TaskType.TYPE_DELIVERY
                delivery = Delivery()
                delivery.pickup_place_name = desc["pickup_place_name"]
                delivery.pickup_dispenser = desc["pickup_dispenser"]
                delivery.dropoff_ingestor = desc["dropoff_ingestor"]
                delivery.dropoff_place_name = desc["dropoff_place_name"]
                req_msg.description.delivery = delivery
            else:
                raise Exception("Invalid TaskType")

            # Calc start time, convert min to sec: TODO better representation
            rclpy.spin_once(self, timeout_sec=0.0)
            ros_start_time = self.get_clock().now().to_msg()
            ros_start_time.sec += int(task_json["start_time"] * 60)
            req_msg.description.start_time = ros_start_time
        except KeyError as ex:
            return None, f"Missing Key value in json body: {ex}"
        except Exception as ex:
            return None, str(ex)
        return req_msg, ""

    def get_task_status(self):
        """
        Get all task status - This fn will trigger a ros srv call to acquire
        all submitted tasks to dispatcher node. Fn returns an object of tasks
        """
        req = GetTaskList.Request()
        try:
            future = self.get_tasks_srv.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            response = future.result()
            if response is None:
                raise Exception("Unable to retrieve tasks")
            else:
                active_tasks = self.__convert_task_status_msg(
                    response.active_tasks, False
                )
                terminated_tasks = self.__convert_task_status_msg(
                    response.terminated_tasks, True
                )
                return {
                    "active_tasks": active_tasks,
                    "terminated_tasks": terminated_tasks,
                }
        except Exception as e:
            self.get_logger().error("Error! GetTasks Srv failed %r" % (e,))
        return []

    def cancel_task_request(self, task_id) -> bool:
        """
        Cancel Task - This function will trigger a ros srv call to the
        dispatcher node, and return a response.
        """
        req = CancelTask.Request()
        req.task_id = task_id
        try:
            future = self.cancel_task_srv.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            response = future.result()
            if response is None:
                self.get_logger().warn("/cancel_task srv call failed")
            else:
                return response.success
        except Exception as e:
            self.get_logger().error("Error! Cancel Srv failed %r" % (e,))
        return False

    def __convert_task_status_msg(self, task_summaries, is_done=True):
        """
        convert task summary msg and return a jsonify-able task status obj
        """
        states_enum = {
            0: "Queued",
            1: "Active/Executing",
            2: "Completed",
            3: "Failed",
            4: "Cancelled",
            5: "Pending",
        }
        type_enum = {
            0: "Station",
            1: "Loop",
            2: "Delivery",
            3: "Charging",
            4: "Clean",
            5: "Patrol",
        }

        status_list = []
        rclpy.spin_once(self, timeout_sec=0.0)
        now = self.get_clock().now().to_msg().sec  # only use sec
        for task in task_summaries:
            desc = task.task_profile.description
            status = StatusModel()
            status.task_id = task.task_id
            status.state = states_enum[task.state]
            status.done = is_done
            status.fleet_name = task.fleet_name
            status.robot_name = task.robot_name
            status.task_type = type_enum[desc.task_type.type]
            status.priority = desc.priority.value
            status.submited_start_time = desc.start_time.sec
            status.start_time = task.start_time.sec  # only use sec
            status.end_time = task.end_time.sec  # only use sec

            if status.task_type == "Clean":
                status.description = desc.clean.start_waypoint
            elif status.task_type == "Loop":
                status.description = (
                    desc.loop.start_name
                    + " --> "
                    + desc.loop.finish_name
                    + " x"
                    + str(desc.loop.num_loops)
                )
            elif status.task_type == "Delivery":
                status.description = (
                    desc.delivery.pickup_place_name
                    + " --> "
                    + desc.delivery.dropoff_place_name
                )
            elif status.task_type == "Charging":
                status.description = "Back to Charging Station"

            # Generate a progress percentage
            duration = abs(task.end_time.sec - task.start_time.sec)
            # check if is completed
            if is_done or task.state == TaskSummary.STATE_FAILED:
                status.progress = "100%"
            # check if it state is queued/cancelled
            elif duration == 0 or (task.state in [0, 4]):
                status.progress = "0%"
            else:
                percent = int(100 * (now - task.start_time.sec) / float(duration))
                if percent < 0:
                    status.progress = "0%"
                elif percent > 100:
                    status.progress = "Delayed"
                else:
                    status.progress = f"{percent}%"
            status_list.insert(0, jsonable_encoder(status))
        return status_list
