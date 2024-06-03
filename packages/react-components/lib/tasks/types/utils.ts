import { TaskRequest } from 'api-client';
import { DefaultPatrolTaskDefinition, makePatrolTaskShortDescription } from './patrol';
import {
  DefaultDeliveryAreaPickupTaskDefinition,
  DefaultDeliveryPickupTaskDefinition,
  DefaultDeliverySequentialLotPickupTaskDefinition,
} from './delivery-custom';
import { getTaskBookingLabelFromTaskRequest } from '../task-booking-label-utils';
import {
  DefaultComposeCleanTaskDefinition,
  makeComposeCleanTaskShortDescription,
} from './compose-clean';
import { DefaultDeliveryTaskDefinition, makeDeliveryTaskShortDescription } from './delivery';
import {
  makeDeliveryPickupTaskShortDescription,
  makeDeliveryCustomTaskShortDescription,
} from './delivery-custom';
import {
  DefaultCustomComposeTaskDefinition,
  makeCustomComposeTaskShortDescription,
} from './custom-compose';

export function isNonEmptyString(value: string): boolean {
  return value.length > 0;
}

export function isPositiveNumber(value: number): boolean {
  return value > 0;
}

function rawStringFromJsonRequest(taskRequest: TaskRequest): string | undefined {
  try {
    const requestString = JSON.stringify(taskRequest);
    console.error(
      `Task does not have a identifying label, failed to generate short description of task: ${requestString}`,
    );
    return requestString;
  } catch (e) {
    console.error(
      `Failed to parse description of task of category: ${taskRequest.category}: ${
        (e as Error).message
      }`,
    );
    return undefined;
  }
}

export function getShortDescription(
  taskRequest: TaskRequest,
  taskDisplayName?: string,
): string | undefined {
  const bookingLabel = getTaskBookingLabelFromTaskRequest(taskRequest);
  if (!bookingLabel) {
    return rawStringFromJsonRequest(taskRequest);
  }

  const taskDefinitionId = bookingLabel.description.task_definition_id;
  switch (taskDefinitionId) {
    case DefaultPatrolTaskDefinition.taskDefinitionId:
      return makePatrolTaskShortDescription(taskRequest.description, taskDisplayName);
    case DefaultDeliveryTaskDefinition.taskDefinitionId:
      return makeDeliveryTaskShortDescription(taskRequest.description, taskDisplayName);
    case DefaultComposeCleanTaskDefinition.taskDefinitionId:
      return makeComposeCleanTaskShortDescription(taskRequest.description, taskDisplayName);
    case DefaultDeliveryPickupTaskDefinition.taskDefinitionId:
      return makeDeliveryPickupTaskShortDescription(taskRequest.description, taskDisplayName);
    case DefaultDeliverySequentialLotPickupTaskDefinition.taskDefinitionId:
    case DefaultDeliveryAreaPickupTaskDefinition.taskDefinitionId:
      return makeDeliveryCustomTaskShortDescription(taskRequest.description, taskDisplayName);
    case DefaultCustomComposeTaskDefinition.taskDefinitionId:
      return makeCustomComposeTaskShortDescription(taskRequest.description);
    default:
      return `[Unknown] type "${taskRequest.description.category}"`;
  }
}
