import { ItemSummary } from 'react-components';

export interface HealthStatus {
  door: ItemSummary;
  lift: ItemSummary;
  dispenser: ItemSummary;
  robot: ItemSummary;
}

export default class RmfHealthStateManager {
  getDoorSummary = (): ItemSummary => {
    return {
      item: 'Door',
      summary: { operational: 2, outOfOrder: 1 },
      spoiltItemList: [
        { name: 'hardware_door', type: 'door', itemNameAndState: 'hardware_door - unknown' },
      ],
    };
  };

  getLiftSummary = (): ItemSummary => {
    return {
      item: 'Lift',
      summary: { operational: 2, outOfOrder: 0 },
      spoiltItemList: [],
    };
  };

  getDispenserSummary = (): ItemSummary => {
    return {
      item: 'Dispensers',
      summary: { operational: 1, outOfOrder: 0 },
      spoiltItemList: [],
    };
  };

  getRobotSummary = (): ItemSummary => {
    return {
      item: 'Robots',
      summary: {
        operational: 3,
        outOfOrder: 0,
        charging: 2,
        idle: 1,
      },
      spoiltItemList: [],
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
