import type {
  Clean,
  Delivery,
  Loop,
  Station,
  SubmitTask,
  Task,
  TaskDescription,
  TaskProfile,
  TaskSummary,
} from 'api-client';
import { TaskSummary as RmfTaskSummary, TaskType as RmfTaskType } from 'rmf-models';

const baseCleanDesc: Clean = {
  start_waypoint: '',
};

export function makeClean(clean: Partial<Clean> = {}): Clean {
  return { ...baseCleanDesc, ...clean };
}

const baseDeliveryDesc: Delivery = {
  task_id: '',
  pickup_behavior: {
    name: '',
    parameters: [],
  },
  pickup_dispenser: '',
  pickup_place_name: '',
  dropoff_behavior: {
    name: '',
    parameters: [],
  },
  dropoff_ingestor: '',
  dropoff_place_name: '',
  items: [],
};

export function makeDelivery(delivery: Partial<Delivery> = {}): Delivery {
  return { ...baseDeliveryDesc, ...delivery };
}

const baseLoopDesc: Loop = {
  task_id: '',
  start_name: '',
  finish_name: '',
  num_loops: 0,
  robot_type: '',
};

export function makeLoop(loop: Partial<Loop> = {}): Loop {
  return { ...baseLoopDesc, ...loop };
}

const baseStationDesc: Station = {
  task_id: '',
  place_name: '',
  robot_type: '',
};

export function makeStation(station: Partial<Station> = {}): Station {
  return { ...baseStationDesc, ...station };
}

const baseTaskDescription: TaskDescription = {
  clean: baseCleanDesc,
  delivery: baseDeliveryDesc,
  loop: baseLoopDesc,
  station: baseStationDesc,
  task_type: { type: 0 },
  priority: { value: 0 },
  start_time: { sec: 0, nanosec: 0 },
};

export function makeTaskDescription(
  taskDescription: Partial<TaskDescription> = {},
): TaskDescription {
  return { ...baseTaskDescription, ...taskDescription };
}

const baseTaskProfile: TaskProfile = {
  task_id: '',
  submission_time: { sec: 0, nanosec: 0 },
  description: baseTaskDescription,
};

export function makeTaskProfile(taskProfile: Partial<TaskProfile> = {}): TaskProfile {
  return {
    ...baseTaskProfile,
    ...taskProfile,
  };
}

const baseTaskSummary: TaskSummary = {
  task_id: '',
  fleet_name: '',
  robot_name: '',
  task_profile: baseTaskProfile,
  end_time: { sec: 0, nanosec: 0 },
  start_time: { sec: 0, nanosec: 0 },
  state: 1,
  status: '',
  submission_time: { sec: 0, nanosec: 0 },
};

export function makeTaskSummary(taskSummary: Partial<TaskSummary> = {}): TaskSummary {
  return {
    ...baseTaskSummary,
    ...taskSummary,
  };
}

export function makeTask(id: string, numberOfPhases: number, currentPhase: number): TaskSummary {
  let status = '';
  for (let i = 0; i < numberOfPhases; i++) {
    if (currentPhase === i + 1) {
      status += '*';
    }
    status += `Phase ${i + 1}\ntest phase ${i + 1}\n\n`;
  }
  status = status.trimEnd();
  return makeTaskSummary({
    task_id: id,
    state: RmfTaskSummary.STATE_ACTIVE,
    status: status,
    fleet_name: 'test_fleet',
    robot_name: 'test_robot',
  });
}

export function makeSubmitTask(): SubmitTask {
  return {
    description: {
      cleaning_zone: 'zone',
    },
    start_time: Math.floor(Date.now() / 1000),
    task_type: RmfTaskType.TYPE_CLEAN,
    priority: 0,
  };
}

export function makeDefinedTask(
  type: string,
  robotName: string,
  id: string,
  numberOfPhases: number,
  currentPhase: number,
): Task {
  let status = '';
  for (let i = 0; i < numberOfPhases; i++) {
    if (currentPhase === i + 1) {
      status += '*';
    }
    status += `Phase ${i + 1}\ntest phase ${i + 1}\n\n`;
  }
  status = status.trimEnd();

  const loop = makeLoop({
    task_id: id,
    robot_type: 'test_robot',
    num_loops: 1,
    start_name: 'loop_location_1',
    finish_name: 'loop_location_2',
  });

  const loopTask = makeTaskDescription({
    start_time: { sec: 0, nanosec: 0 },
    priority: { value: 0 },
    task_type: { type: 1 },
    loop: loop,
  });

  const delivery = makeDelivery({
    task_id: id,
    items: [],
    pickup_place_name: 'pickup_1',
    pickup_dispenser: 'pickup_dispenser',
    dropoff_place_name: 'dropoff_1',
    dropoff_ingestor: 'dropoff_ingesstor',
  });

  const deliveryTask = makeTaskDescription({
    start_time: { sec: 0, nanosec: 0 },
    priority: { value: 0 },
    task_type: { type: 2 },
    delivery: delivery,
  });

  const clean = makeClean({
    start_waypoint: 'cleaning_zone',
  });

  const cleaningTask = makeTaskDescription({
    start_time: { sec: 0, nanosec: 0 },
    priority: { value: 0 },
    task_type: { type: 4 },
    clean: clean,
  });

  function checkType() {
    switch (type) {
      case 'Delivery':
        return deliveryTask;

      case 'Loop':
        return loopTask;

      case 'Clean':
        return cleaningTask;

      default:
        return loopTask;
    }
  }

  const taskSummary = makeTaskSummary({
    task_id: id,
    task_profile: {
      task_id: id,
      submission_time: { sec: 0, nanosec: 0 },
      description: checkType(),
    },
    start_time: { sec: 0, nanosec: 0 },
    end_time: { sec: 0, nanosec: 0 },
    submission_time: { sec: 0, nanosec: 0 },
    state: RmfTaskSummary.STATE_ACTIVE,
    status: status,
    fleet_name: 'test_fleet',
    robot_name: robotName,
  });

  const progress = `${Math.floor(Math.random() * 100)}%`;

  return {
    task_id: id,
    authz_grp: 'test_group',
    summary: taskSummary,
    progress: { status: progress },
  };
}
