import { TaskRequest } from 'api-client';
import {
  DefaultPatrolTaskDefinition,
  makePatrolTaskShortDescription,
  makeDefaultPatrolTaskDescription,
} from './patrol';
import {
  DefaultDeliveryAreaPickupTaskDefinition,
  DefaultDeliveryPickupTaskDefinition,
  DefaultDeliverySequentialLotPickupTaskDefinition,
  makeDeliveryPickupTaskShortDescription,
  makeDeliveryCustomTaskShortDescription,
  makeDefaultDeliveryCustomTaskDescription,
  makeDefaultDeliveryPickupTaskDescription,
} from './delivery-custom';
import { getTaskBookingLabelFromTaskRequest } from '../task-booking-label-utils';
import {
  DefaultComposeCleanTaskDefinition,
  makeComposeCleanTaskShortDescription,
  makeDefaultComposeCleanTaskDescription,
} from './compose-clean';
import {
  DefaultDeliveryTaskDefinition,
  makeDeliveryTaskShortDescription,
  makeDefaultDeliveryTaskDescription,
} from './delivery';
import {
  DefaultCustomComposeTaskDefinition,
  makeCustomComposeTaskShortDescription,
} from './custom-compose';
import { TaskDescription } from '../create-task';

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

export function getDefaultTaskDescription(
  taskDefinitionId: string,
): TaskDescription | string | undefined {
  switch (taskDefinitionId) {
    case DefaultComposeCleanTaskDefinition.taskDefinitionId:
      return makeDefaultComposeCleanTaskDescription();
    case DefaultDeliveryPickupTaskDefinition.taskDefinitionId:
      return makeDefaultDeliveryPickupTaskDescription();
    case DefaultDeliverySequentialLotPickupTaskDefinition.taskDefinitionId:
    case DefaultDeliveryAreaPickupTaskDefinition.taskDefinitionId:
      return makeDefaultDeliveryCustomTaskDescription(taskDefinitionId);
    case DefaultDeliveryTaskDefinition.taskDefinitionId:
      return makeDefaultDeliveryTaskDescription();
    case DefaultPatrolTaskDefinition.taskDefinitionId:
      return makeDefaultPatrolTaskDescription();
    case DefaultCustomComposeTaskDefinition.taskDefinitionId:
      return '';
    default:
      return undefined;
  }
}

export function getTaskRequestCategory(taskDefinitionId: string): string | undefined {
  switch (taskDefinitionId) {
    case DefaultComposeCleanTaskDefinition.taskDefinitionId:
      return DefaultComposeCleanTaskDefinition.requestCategory;
    case DefaultDeliveryPickupTaskDefinition.taskDefinitionId:
      return DefaultDeliveryPickupTaskDefinition.requestCategory;
    case DefaultDeliverySequentialLotPickupTaskDefinition.taskDefinitionId:
      return DefaultDeliverySequentialLotPickupTaskDefinition.requestCategory;
    case DefaultDeliveryAreaPickupTaskDefinition.taskDefinitionId:
      return DefaultDeliveryAreaPickupTaskDefinition.requestCategory;
    case DefaultDeliveryTaskDefinition.taskDefinitionId:
      return DefaultDeliveryTaskDefinition.requestCategory;
    case DefaultPatrolTaskDefinition.taskDefinitionId:
      return DefaultPatrolTaskDefinition.requestCategory;
    case DefaultCustomComposeTaskDefinition.taskDefinitionId:
      return DefaultCustomComposeTaskDefinition.requestCategory;
  }
  return undefined;
}
