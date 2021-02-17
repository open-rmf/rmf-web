import { DoorSummary, LiftSummary, DispenserSummary, RobotSummary } from 'react-components';

export interface HealthStatus {
  door: DoorSummary;
  lift: LiftSummary;
  dispenser: DispenserSummary;
  robot: RobotSummary;
}

export default class RmfHealthStateManager {
  getDoorSummary = (): DoorSummary => {
    return {
      item: 'Door',
      itemSummary: { operational: 2, outOfOrder: 1 },
      spoiltDoors: [
        {
          itemNameAndState: 'hardware_door - unknown',
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
      item: 'Lift',
      itemSummary: { operational: 2, outOfOrder: 0 },
      spoiltLifts: [],
    };
  };

  getDispenserSummary = (): DispenserSummary => {
    return {
      item: 'Dispensers',
      itemSummary: { operational: 1, outOfOrder: 0 },
      spoiltDispensers: [],
    };
  };

  getRobotSummary = (): RobotSummary => {
    return {
      item: 'Robots',
      robotSummary: {
        operational: 3,
        outOfOrder: 0,
        charging: 2,
        idle: 1,
      },
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
