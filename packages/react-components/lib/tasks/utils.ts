import { TaskType as RmfTaskType } from 'rmf-models';
import type { TaskState, TaskRequest } from 'api-client';

export function taskTypeToStr(taskType: number): string {
  switch (taskType) {
    case RmfTaskType.TYPE_CHARGE_BATTERY:
      return 'Charge';
    case RmfTaskType.TYPE_CLEAN:
      return 'Clean';
    case RmfTaskType.TYPE_DELIVERY:
      return 'Delivery';
    case RmfTaskType.TYPE_LOOP:
      return 'Loop';
    case RmfTaskType.TYPE_PATROL:
      return 'Patrol';
    case RmfTaskType.TYPE_STATION:
      return 'Station';
    default:
      return 'Unknown';
  }
}

function parsePhaseDetail(phases: TaskState['phases'], category?: string) {
  if (phases) {
    if (category === 'Loop') {
      const startPhase = phases['1'];
      const endPhase = phases['2'];
      const from = startPhase.category?.split('[place:')[1].split(']')[0];
      const to = endPhase.category?.split('[place:')[1].split(']')[0];
      return { to, from };
    }
  }
  return {};
}

export function parseTaskDetail(
  task: TaskState,
  category?: string,
): { to: string; from: string } | {} {
  if (category?.includes('Loop')) return parsePhaseDetail(task.phases, category);
  if (category?.includes('Delivery')) {
    const from = category?.split('[place:')[1].split(']')[0];
    const to = category?.split('[place:')[2].split(']')[0];
    return { to, from };
  } else {
    return {};
  }
}

export function parsePickup(state: TaskState, request?: TaskRequest): string {
  if (request === undefined || request.category.toLowerCase() === 'patrol') {
    return 'n/a';
  }

  // custom deliveries
  const supportedDeliveries = [
    'delivery_pickup',
    'delivery_sequential_lot_pickup',
    'delivery_area_pickup',
  ];
  if (
    !request.description['category'] ||
    !supportedDeliveries.includes(request.description['category'])
  ) {
    return 'n/a';
  }

  // TODO(ac): use schemas
  try {
    const deliveryType: string = request.description['category'];
    const perform_action_description =
      request.description['phases'][0]['activity']['description']['activities'][1]['description'][
        'description'
      ];

    switch (deliveryType) {
      case 'delivery_pickup': {
        const pickup_lot: string = perform_action_description['pickup_lot'];
        return pickup_lot;
      }
      case 'delivery_sequential_lot_pickup':
      case 'delivery_area_pickup': {
        const pickup_zone: string = perform_action_description['pickup_zone'];
        return pickup_zone;
      }
      default:
        return 'n/a';
    }
  } catch (e) {
    console.error(`Failed to parse pickup lot/zone from task request: ${(e as Error).message}`);
  }

  return 'n/a';
}

export function parseDestination(state: TaskState, request: TaskRequest): string {
  if (request === undefined) {
    return 'n/a';
  }

  // patrol
  if (
    request.category.toLowerCase() === 'patrol' &&
    request.description['places'] !== undefined &&
    request.description['places'].length > 0
  ) {
    return request.description['places'].at(-1);
  }

  // custom deliveries
  const supportedDeliveries = [
    'delivery_pickup',
    'delivery_sequential_lot_pickup',
    'delivery_area_pickup',
  ];
  if (
    !request.description['category'] ||
    !supportedDeliveries.includes((request.description['category'] as string).toLowerCase())
  ) {
    return 'n/a';
  }

  // TODO(ac): use schemas
  try {
    const destination =
      request.description['phases'][0]['activity']['description']['activities'][2]['description'];
    return destination;
  } catch (e) {
    console.error(`Failed to parse destination from task request: ${(e as Error).message}`);
  }

  // automated tasks that can only be parsed with state
  if (state.category && state.category === 'Charge Battery') {
    try {
      const charge_phase = state['phases'] ? state['phases']['1'] : undefined;
      const charge_events = charge_phase ? charge_phase.events : undefined;
      const charge_event_name = charge_events ? charge_events['1'].name : undefined;
      if (charge_event_name === undefined) {
        console.error('Unable to parse charging event name.');
        return 'n/a';
      }

      const charging_place = charge_event_name?.split('[place:')[1].split(']')[0];
      return charging_place;
    } catch (e) {
      console.error(`Failed to parse charging point from task state: ${(e as Error).message}`);
    }
  }

  return 'n/a';
}
