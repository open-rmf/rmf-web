import asyncio
import os
from typing import Optional

from tortoise import Tortoise

import api_server.models.tortoise_models as ttm
from api_server.app_config import app_config, load_config
from api_server.models import TaskRequest, TaskRequestLabel, TaskState

# NOTE: This script is for migrating TaskState in an existing database to work
# with https://github.com/open-rmf/rmf-web/pull/912.
# Before migration:
# - Pickup, destination, cart ID, category information will be unavailable on
#   the Task Queue Table on the dashboard, as we no longer gather those
#   fields from the TaskRequest
# After migration:
# - Dashboard will behave the same as before #912, however it is no longer
#   dependent on TaskRequest to fill out those fields. It gathers those fields
#   from the json string in TaskState.booking.labels.
# This script performs the following:
# - Construct TaskRequestLabel from its TaskRequest if it is available.
# - Update the respective TaskState.data json TaskState.booking.labels field
#   with the newly constructed TaskRequestLabel json string.
# - Update ScheduledTask to use labels too


app_config = load_config(
    os.environ.get(
        "RMF_API_SERVER_CONFIG",
        f"{os.path.dirname(__file__)}/../../default_config.py",
    )
)


def parse_category(task_request: TaskRequest) -> Optional[str]:
    category = None
    if task_request.category.lower() == "patrol":
        category = "Patrol"
    elif task_request.description and task_request.description["category"]:
        category = task_request.description["category"]
    return category


def parse_pickup(task_request: TaskRequest) -> Optional[str]:
    # patrol
    if task_request.category.lower() == "patrol":
        return None

    # custom deliveries
    supportedDeliveries = [
        "delivery_pickup",
        "delivery_sequential_lot_pickup",
        "delivery_area_pickup",
    ]
    if (
        "category" not in task_request.description
        or task_request.description["category"] not in supportedDeliveries
    ):
        return None

    category = task_request.description["category"]
    try:
        perform_action_description = task_request.description["phases"][0]["activity"][
            "description"
        ]["activities"][1]["description"]["description"]
        if category == "delivery_pickup":
            return perform_action_description["pickup_lot"]
        return perform_action_description["pickup_zone"]
    except Exception as e:  # pylint: disable=W0703
        print(f"Failed to parse pickup for task of category {category}")
    return None


def parse_destination(task_request: TaskRequest) -> Optional[str]:
    # patrol
    try:
        if (
            task_request.category.lower() == "patrol"
            and task_request.description["places"] is not None
            and len(task_request.description["places"]) > 0
        ):
            return task_request.description["places"][-1]
    except Exception as e:  # pylint: disable=W0703
        print("Failed to parse destination for patrol")
        return None

    # custom deliveries
    supportedDeliveries = [
        "delivery_pickup",
        "delivery_sequential_lot_pickup",
        "delivery_area_pickup",
    ]
    if (
        "category" not in task_request.description
        or task_request.description["category"] not in supportedDeliveries
    ):
        return None

    category = task_request.description["category"]
    try:
        destination = task_request.description["phases"][1]["activity"]["description"][
            "activities"
        ][0]["description"]
        return destination
    except Exception as e:  # pylint: disable=W0703
        print(f"Failed to parse destination from task request of category {category}")
    return None


def parse_cart_id(task_request: TaskRequest) -> Optional[str]:
    # patrol
    if task_request.category.lower() == "patrol":
        return None

    # custom deliveries
    supportedDeliveries = [
        "delivery_pickup",
        "delivery_sequential_lot_pickup",
        "delivery_area_pickup",
    ]
    if (
        "category" not in task_request.description
        or task_request.description["category"] not in supportedDeliveries
    ):
        return None

    category = task_request.description["category"]
    try:
        perform_action_description = task_request.description["phases"][0]["activity"][
            "description"
        ]["activities"][1]["description"]["description"]
        return perform_action_description["cart_id"]
    except Exception as e:  # pylint: disable=W0703
        print(f"Failed to parse cart ID for task of category {category}")
    return None


async def migrate():
    await Tortoise.init(
        db_url=app_config.db_url,
        modules={"models": ["api_server.models.tortoise_models"]},
    )
    await Tortoise.generate_schemas()

    # Acquire all existing TaskStates
    states = await ttm.TaskState.all()
    print(f"Migrating {len(states)} TaskState models")

    # Migrate each TaskState
    for state in states:
        state_model = TaskState(**state.data)
        task_id = state_model.booking.id

        # If the request is not available we skip migrating this TaskState
        request = await ttm.TaskRequest.get_or_none(id_=task_id)
        if request is None:
            continue
        request_model = TaskRequest(**request.request)

        # Construct TaskRequestLabel based on TaskRequest
        pickup = parse_pickup(request_model)
        destination = parse_destination(request_model)
        label = TaskRequestLabel(
            category=parse_category(request_model),
            unix_millis_warn_time=None,
            pickup=pickup,
            destination=destination,
            cart_id=parse_cart_id(request_model),
        )
        print(label)

        # Update data json
        if state_model.booking.labels is None:
            state_model.booking.labels = [label.json()]
        else:
            state_model.booking.labels.append(label.json())
        print(state_model)

        state.update_from_dict(
            {
                "data": state_model.json(),
                "pickup": pickup,
                "destination": destination,
            }
        )
        await state.save()

    # Acquire all ScheduledTask
    scheduled_tasks = await ttm.ScheduledTask.all()
    print(f"Migrating {len(scheduled_tasks)} ScheduledTask models")

    # Migrate each ScheduledTask
    for scheduled_task in scheduled_tasks:
        scheduled_task_model = await ttm.ScheduledTaskPydantic.from_tortoise_orm(
            scheduled_task
        )
        task_request = TaskRequest(**scheduled_task_model.task_request)
        print(task_request)

        # Construct TaskRequestLabel based on TaskRequest
        pickup = parse_pickup(task_request)
        destination = parse_destination(task_request)
        label = TaskRequestLabel(
            category=parse_category(task_request),
            unix_millis_warn_time=None,
            pickup=pickup,
            destination=destination,
            cart_id=parse_cart_id(task_request),
        )
        print(label)

        # Update TaskRequest
        if task_request.labels is None:
            task_request.labels = [label.json()]
        else:
            task_request.labels.append(label.json())
        print(task_request)

        # Update ScheduledTask
        scheduled_task.update_from_dict({"task_request": task_request.json()})
        await scheduled_task.save()

    await Tortoise.close_connections()


def main():
    print("Migration started")
    asyncio.run(migrate())
    print("Migration done")


if __name__ == "__main__":
    main()
