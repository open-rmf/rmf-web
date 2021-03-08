import { DoorSummary, DispenserSummary, LiftSummary, RobotSummary } from 'react-components';

export interface HealthStatus {
  door: DoorSummary;
  lift: LiftSummary;
  dispenser: DispenserSummary;
  robot: RobotSummary;
}

// TODO - fill up health state manager logic once backend is up
export default class RmfHealthStateManager {
  // FIXME - remove mock values
  getDoorSummary = (): DoorSummary => {
    return {
      operational: 2,
      spoiltItem: [
        {
          name: 'hardware_door',
          state: 'unknown',
          door: {
            name: 'hardware_door',
            v1_x: 4.9,
            v1_y: -4,
            v2_x: 4.4,
            v2_y: -5,
            door_type: 1,
            motion_range: 1.571,
            motion_direction: -1,
          },
        },
      ],
    };
  };

  getLiftSummary = (): LiftSummary => {
    return {
      operational: 2,
      spoiltItem: [],
    };
  };

  getDispenserSummary = (): DispenserSummary => {
    return {
      operational: 1,
      spoiltItem: [],
    };
  };

  getRobotSummary = (): RobotSummary => {
    return {
      operational: 3,
      charging: 2,
      idle: 1,
      spoiltRobots: [],
    };
  };

  getHealthStatus = (): HealthStatus => {
    const getDoor = this.getDoorSummary();
    const getLift = this.getLiftSummary();
    const getDispensers = this.getDispenserSummary();
    const getRobots = this.getRobotSummary();
    return {
      door: getDoor,
      lift: getLift,
      dispenser: getDispensers,
      robot: getRobots,
    };
  };
}
