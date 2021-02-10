import { ItemSummary, liftModeToString, robotModeToString, SpoiltItem } from 'react-components';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

export interface HealthStatus {
  door: ItemSummary;
  lift: ItemSummary;
  dispenser: ItemSummary;
  robot: ItemSummary;
}

export interface ItemState {
  doors: Record<string, RomiCore.DoorState>;
  lifts: Record<string, RomiCore.LiftState>;
  robots: Record<string, RomiCore.FleetState>;
  dispensers: Record<string, RomiCore.DispenserState>;
}

export default class RmfHealthStateManager {
  itemState: ItemState;

  constructor(itemState: ItemState) {
    this.itemState = itemState;
  }

  getDoorSummary = (): ItemSummary => {
    let modeCounter = { operational: 0, outOfOrder: 0 };
    const spoiltEquipment: SpoiltItem[] = [];
    const doors = this.itemState.doors;
    const doorKeys = Object.keys(doors);

    doorKeys.forEach((door) => {
      switch (doors[door].current_mode.value) {
        case RomiCore.DoorMode.MODE_CLOSED:
        case RomiCore.DoorMode.MODE_MOVING:
        case RomiCore.DoorMode.MODE_OPEN:
          modeCounter.operational += 1;
          break;
        default:
          modeCounter.outOfOrder += 1;
          spoiltEquipment.push({ type: 'door', name: door, itemNameAndState: `${door} - Unknown` });
          break;
      }
    });
    return {
      item: 'Door',
      summary: { operational: modeCounter.operational, outOfOrder: modeCounter.outOfOrder },
      spoiltItemList: spoiltEquipment,
    };
  };

  getLiftSummary = (): ItemSummary => {
    let modeCounter = { operational: 0, outOfOrder: 0 };
    const spoiltEquipment: SpoiltItem[] = [];
    const lifts = this.itemState.lifts;
    const liftKeys = Object.keys(lifts);

    liftKeys.forEach((lift) => {
      switch (lifts[lift].current_mode) {
        case RomiCore.LiftState.MODE_HUMAN:
        case RomiCore.LiftState.MODE_AGV:
        case RomiCore.LiftState.MODE_OFFLINE:
          modeCounter.operational += 1;
          break;
        case RomiCore.LiftState.MODE_FIRE:
        case RomiCore.LiftState.MODE_EMERGENCY:
        case RomiCore.LiftState.MODE_UNKNOWN:
          spoiltEquipment.push({
            type: 'lift',
            name: lift,
            itemNameAndState: lift + ` - ${liftModeToString(lifts[lift].current_mode)}`,
          });
          modeCounter.outOfOrder += 1;
          break;
      }
    });
    return {
      item: 'Lift',
      summary: { operational: modeCounter.operational, outOfOrder: modeCounter.outOfOrder },
      spoiltItemList: spoiltEquipment,
    };
  };

  getDispenserSummary = (): ItemSummary => {
    let modeCounter = { operational: 0, outOfOrder: 0 };
    const spoiltEquipment: SpoiltItem[] = [];
    const dispensers = this.itemState.dispensers;
    const dispenserKeys = Object.keys(dispensers);

    dispenserKeys.forEach((dispenser) => {
      switch (dispensers[dispenser].mode) {
        case RomiCore.DispenserState.IDLE:
        case RomiCore.DispenserState.BUSY:
        case RomiCore.DispenserState.OFFLINE:
          modeCounter.operational += 1;
          break;
        default:
          spoiltEquipment.push({
            type: 'dispenser',
            name: dispenser,
            itemNameAndState: dispenser + '- Unknown',
          });
          modeCounter.outOfOrder += 1;
          break;
      }
    });

    return {
      item: 'Dispensers',
      summary: { operational: modeCounter.operational, outOfOrder: modeCounter.outOfOrder },
      spoiltItemList: spoiltEquipment,
    };
  };

  getRobotSummary = (): ItemSummary => {
    let modeCounter = { operational: 0, outOfOrder: 0, charging: 0, idle: 0 };
    const spoiltEquipment: SpoiltItem[] = [];
    const fleets = this.itemState.robots;
    const fleetKeys = Object.keys(fleets);

    fleetKeys.forEach((fleet) => {
      fleets[fleet].robots.forEach((robot) => {
        switch (robot.mode.mode) {
          case RomiCore.RobotMode.MODE_ADAPTER_ERROR:
          case RomiCore.RobotMode.MODE_EMERGENCY:
            spoiltEquipment.push({
              type: 'robot',
              name: robot.name,
              fleet: fleet,
              itemNameAndState: robot.name + ` - ${robotModeToString(robot.mode)}`,
            });
            modeCounter.outOfOrder += 1;
            break;
          case RomiCore.RobotMode.MODE_CHARGING:
            modeCounter.operational += 1;
            modeCounter.charging += 1;
            break;
          case RomiCore.RobotMode.MODE_DOCKING:
          case RomiCore.RobotMode.MODE_GOING_HOME:
          case RomiCore.RobotMode.MODE_MOVING:
          case RomiCore.RobotMode.MODE_PAUSED:
          case RomiCore.RobotMode.MODE_WAITING:
            modeCounter.operational += 1;
            break;
          case RomiCore.RobotMode.MODE_IDLE:
            modeCounter.operational += 1;
            modeCounter.idle += 1;
            break;
        }
      });
    });
    return {
      item: 'Robots',
      summary: {
        operational: modeCounter.operational,
        outOfOrder: modeCounter.outOfOrder,
        charging: modeCounter.charging,
        idle: modeCounter.idle,
      },
      spoiltItemList: spoiltEquipment,
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
