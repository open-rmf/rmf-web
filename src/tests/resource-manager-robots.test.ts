import { RobotResourceManager, RobotResource } from '../resource-manager-robots';
import fakeResources from '../mock/data/resources';

describe('Correct functionally of getAvailablePlacesPerFleet', () => {
  let manager: RobotResourceManager;
  let resourceData: Record<string, RobotResource>;

  beforeAll(() => {
    resourceData = fakeResources().robots;
    manager = new RobotResourceManager(resourceData);
  });

  test('FleetName does not exists', () => {
    const places = manager.getAvailablePlacesPerFleet('Not exists');
    expect(places).toBeFalsy();
  });

  test('FleetName exists with places', () => {
    const fleetName = Object.keys(resourceData)[0]; // tinyRobot
    const places = manager.getAvailablePlacesPerFleet(fleetName) as String[];
    expect(Object.keys(places).length).toEqual(Object.keys(resourceData[fleetName].places).length);
  });

  test('FleetName exists without places', () => {
    const fleetName = Object.keys(resourceData)[1]; // deliveryRobot
    const places = manager.getAvailablePlacesPerFleet(fleetName) as String[];
    expect(Object.keys(places).length).toBeFalsy();
  });
});

describe('Correct functionally of getIconPath', () => {
  let manager: RobotResourceManager;
  let resourceData: Record<string, RobotResource>;

  beforeAll(() => {
    resourceData = fakeResources().robots;
    manager = new RobotResourceManager(resourceData);
  });

  test('FleetName does not exists', () => {
    const icon = manager.getIconPath('Not exists');
    expect(icon).toBeFalsy();
  });

  test('Icon its empty', () => {
    resourceData.testFleet = {
      icons: {
        testFleet: '',
      },
      places: {},
    };
    const icon = manager.getIconPath('testFleet');
    expect(icon).toBeFalsy();
  });

  test('Pick model over fleetName', () => {
    resourceData.testFleet = {
      icons: {
        testFleet: '/fleet',
        testModel: '/model',
      },
      places: {},
    };
    const icon = manager.getIconPath('testFleet', 'testModel');
    expect(icon).toEqual('/assets/icons/model');
  });

  test('It has an icon', () => {
    const fleetName = Object.keys(resourceData)[0]; // tinyRobot
    const icon = manager.getIconPath(fleetName);
    expect(icon).toBeTruthy();
  });
});

describe('Correct functionally of getDispensersPerFleet', () => {
  let manager: RobotResourceManager;
  let resourceData: Record<string, RobotResource>;

  beforeAll(() => {
    resourceData = fakeResources().robots;
    manager = new RobotResourceManager(resourceData);
  });

  test('Place exists but FleetName does not', () => {
    const dispenser = manager.getDispensersPerFleet('Not exists', 'supplies');
    expect(dispenser).toBeFalsy();
  });

  test('FleetName exists but Place does not', () => {
    const fleetName = Object.keys(resourceData)[0]; // tinyRobot
    const dispenser = manager.getDispensersPerFleet(fleetName, 'Not exists');
    expect(dispenser).toBeFalsy();
  });

  test('Place has no dispenser', () => {
    const fleetName = Object.keys(resourceData)[0]; // tinyRobot
    const dispensers = manager.getDispensersPerFleet(fleetName, 'supplies');
    expect(dispensers?.length).toBe(0);
  });

  test('Place has dispenser', () => {
    const fleetName = Object.keys(resourceData)[0]; // tinyRobot
    const dispensers = manager.getDispensersPerFleet(fleetName, 'pantry');
    expect(dispensers?.length).toBe(resourceData[fleetName].places.pantry.length);
  });
});
