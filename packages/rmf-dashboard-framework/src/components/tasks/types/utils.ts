import { TaskRequest } from 'api-client';

import {
  getTaskBookingLabelFromTaskRequest,
  getTaskDefinitionId,
} from '../task-booking-label-utils';
import { TaskDefinition, TaskDescription } from '../task-form';
import {
  ComposeCleanTaskDefinition,
  makeComposeCleanTaskShortDescription,
  makeDefaultComposeCleanTaskDescription,
} from './compose-clean';
import {
  CustomComposeTaskDefinition,
  makeCustomComposeTaskShortDescription,
} from './custom-compose';
import {
  DeliveryTaskDefinition,
  makeDefaultDeliveryTaskDescription,
  makeDeliveryTaskShortDescription,
} from './delivery';
import {
  DeliveryAreaPickupTaskDefinition,
  DeliveryPickupTaskDefinition,
  DeliverySequentialLotPickupTaskDefinition,
  makeDefaultDeliveryCustomTaskDescription,
  makeDefaultDeliveryPickupTaskDescription,
  makeDeliveryCustomTaskShortDescription,
  makeDeliveryPickupTaskShortDescription,
} from './delivery-custom';
import {
  makeDefaultPatrolTaskDescription,
  makePatrolTaskShortDescription,
  PatrolTaskDefinition,
} from './patrol';

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

  const taskDefinitionId = getTaskDefinitionId(bookingLabel);
  if (!taskDefinitionId) {
    return undefined;
  }
  switch (taskDefinitionId) {
    case PatrolTaskDefinition.taskDefinitionId:
      return makePatrolTaskShortDescription(taskRequest.description, taskDisplayName);
    case DeliveryTaskDefinition.taskDefinitionId:
      return makeDeliveryTaskShortDescription(taskRequest.description, taskDisplayName);
    case ComposeCleanTaskDefinition.taskDefinitionId:
      return makeComposeCleanTaskShortDescription(taskRequest.description, taskDisplayName);
    case DeliveryPickupTaskDefinition.taskDefinitionId:
      return makeDeliveryPickupTaskShortDescription(taskRequest.description, taskDisplayName);
    case DeliverySequentialLotPickupTaskDefinition.taskDefinitionId:
    case DeliveryAreaPickupTaskDefinition.taskDefinitionId:
      return makeDeliveryCustomTaskShortDescription(taskRequest.description, taskDisplayName);
    case CustomComposeTaskDefinition.taskDefinitionId:
      return makeCustomComposeTaskShortDescription(taskRequest.description);
    default:
      return `[Unknown] type "${taskRequest.description.category}"`;
  }
}

export function getDefaultTaskDefinition(taskDefinitionId: string): TaskDefinition | undefined {
  switch (taskDefinitionId) {
    case ComposeCleanTaskDefinition.taskDefinitionId:
      return ComposeCleanTaskDefinition;
    case DeliveryPickupTaskDefinition.taskDefinitionId:
      return DeliveryPickupTaskDefinition;
    case DeliverySequentialLotPickupTaskDefinition.taskDefinitionId:
      return DeliverySequentialLotPickupTaskDefinition;
    case DeliveryAreaPickupTaskDefinition.taskDefinitionId:
      return DeliveryAreaPickupTaskDefinition;
    case DeliveryTaskDefinition.taskDefinitionId:
      return DeliveryTaskDefinition;
    case PatrolTaskDefinition.taskDefinitionId:
      return PatrolTaskDefinition;
    case CustomComposeTaskDefinition.taskDefinitionId:
      return CustomComposeTaskDefinition;
  }
  return undefined;
}

export function getDefaultTaskDescription(
  taskDefinitionId: string,
): TaskDescription | string | undefined {
  switch (taskDefinitionId) {
    case ComposeCleanTaskDefinition.taskDefinitionId:
      return makeDefaultComposeCleanTaskDescription();
    case DeliveryPickupTaskDefinition.taskDefinitionId:
      return makeDefaultDeliveryPickupTaskDescription();
    case DeliverySequentialLotPickupTaskDefinition.taskDefinitionId:
    case DeliveryAreaPickupTaskDefinition.taskDefinitionId:
      return makeDefaultDeliveryCustomTaskDescription(taskDefinitionId);
    case DeliveryTaskDefinition.taskDefinitionId:
      return makeDefaultDeliveryTaskDescription();
    case PatrolTaskDefinition.taskDefinitionId:
      return makeDefaultPatrolTaskDescription();
    case CustomComposeTaskDefinition.taskDefinitionId:
      return '';
    default:
      return undefined;
  }
}

export function getTaskRequestCategory(taskDefinitionId: string): string | undefined {
  const definition = getDefaultTaskDefinition(taskDefinitionId);
  return definition !== undefined ? definition.requestCategory : undefined;
}
