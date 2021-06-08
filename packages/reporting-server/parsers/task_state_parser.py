import json

from models.task_state import TaskState


async def task_state_parser(fullstring: str) -> list:
    modified_string = fullstring.replace("task_state:", "")
    state_json = json.loads(modified_string)

    if not isinstance(state_json["tasks"], list) or len(state_json["tasks"]) == 0:
        print("tasks should be a list")
        return []

    task_list = []
    for task in state_json["tasks"]:
        task_list.append(
            {
                "payload": modified_string,
                "fleet_name": task["fleet_name"],
                "robot_name": task["robot_name"],
                "task_id": task["task_id"],
                "state": TaskState.service.get_task_state_name(task["state"]),
                "task_type": TaskState.service.get_task_type_name(task["task_type"]),
                "priority": task["priority"],
                "submission_time": task["submission_time"],
                "start_time": task["start_time"],
                "end_time": task["end_time"],
            }
        )

    return task_list
