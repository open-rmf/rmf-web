import { TaskRequest } from 'api-client';
import { GoToPlaceActivity, LotPickupActivity } from './delivery-custom';

export function isNonEmptyString(value: string): boolean {
  return value.length > 0;
}

export function isPositiveNumber(value: number): boolean {
  return value > 0;
}

export function getShortDescription(
  taskRequest: TaskRequest,
  remappedTaskName?: string,
): string | undefined {
  switch (taskRequest.category) {
    case 'patrol': {
      const formattedPlaces = taskRequest.description.places.map((place: string) => `[${place}]`);
      return `[${remappedTaskName ?? 'Patrol'}] [${
        taskRequest.description.rounds
      }] round/s, along ${formattedPlaces.join(', ')}`;
    }
    case 'delivery': {
      return `[${remappedTaskName ?? 'Delivery'}] Pickup [${
        taskRequest.description.pickup.payload.sku
      }] from [${taskRequest.description.pickup.place}], dropoff [${
        taskRequest.description.dropoff.payload.sku
      }] at [${taskRequest.description.dropoff.place}]`;
    }
  }

  // FIXME: This block is only for non-primary task cateogries, that utilize
  // compose.
  if (taskRequest.description.category === 'clean') {
    const cleanActivity = taskRequest.description.phases[0].activity.description.activities[1];
    return `[${remappedTaskName ?? 'Clean'}] zone [${cleanActivity.description.description.zone}]`;
  }

  // This section is only valid for custom delivery types
  // FIXME: This block looks like it makes assumptions about the structure of
  // the task description in order to parse it, but it is following the
  // statically defined description (object) at the top of this file. The
  // descriptions should be replaced by a schema in general, however the better
  // approach now should be to make each task description testable and in charge
  // of their own short descriptions.
  try {
    const goToPickup: GoToPlaceActivity =
      taskRequest.description.phases[0].activity.description.activities[0];
    const pickup: LotPickupActivity =
      taskRequest.description.phases[0].activity.description.activities[1];
    const cartId = pickup.description.description.cart_id;
    const goToDropoff: GoToPlaceActivity =
      taskRequest.description.phases[1].activity.description.activities[0];

    switch (taskRequest.description.category) {
      case 'delivery_pickup': {
        return `[${remappedTaskName ?? 'Delivery - 1:1'}] payload [${cartId}] from [${
          goToPickup.description
        }] to [${goToDropoff.description}]`;
      }
      case 'delivery_sequential_lot_pickup': {
        return `[${
          remappedTaskName ?? 'Delivery - Sequential lot pick up'
        }] payload [${cartId}] from [${goToPickup.description}] to [${goToDropoff.description}]`;
      }
      case 'delivery_area_pickup': {
        return `[${remappedTaskName ?? 'Delivery - Area pick up'}] payload [${cartId}] from [${
          goToPickup.description
        }] to [${goToDropoff.description}]`;
      }
      default:
        return `[Unknown] type "${taskRequest.description.category}"`;
    }
  } catch (e) {
    if (e instanceof TypeError) {
      console.error(`Failed to parse custom delivery: ${e.message}`);
    } else {
      console.error(
        `Failed to generate short description from task of category: ${taskRequest.category}: ${
          (e as Error).message
        }`,
      );
    }

    try {
      const descriptionString = JSON.stringify(taskRequest.description);
      console.error(descriptionString);
      return descriptionString;
    } catch (e) {
      console.error(
        `Failed to parse description of task of category: ${taskRequest.category}: ${
          (e as Error).message
        }`,
      );
      return undefined;
    }
  }
}
