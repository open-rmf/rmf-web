import { RobotResourceManager, RobotResource } from '../resource-manager-robots';
import fakeResources from '../mock/data/resources';
import { DispenserResource, DispenserResourceManager } from '../resource-manager-dispensers';

describe('Correct functionally of getIconPath', () => {
  let manager: DispenserResourceManager;
  let resourceData: Record<string, DispenserResource>;

  beforeAll(() => {
    resourceData = fakeResources().dispensers as Record<string, DispenserResource>;
    manager = new DispenserResourceManager(resourceData);
  });

  test('dispenserName does not exists', () => {
    const icon = manager.getIconPath('Not exists');
    expect(icon).toBeFalsy();
  });

  test('Icon path it`s empty', () => {
    resourceData.testDispenser = {
      icons: {
        testDispenser: '',
      },
      location: { x: 1, y: 1, yaw: 1, level_name: 'L1' },
    };
    const icon = manager.getIconPath('testDispenser');
    expect(icon).toBeFalsy();
  });

  test('It has an icon', () => {
    const dispenserName = Object.keys(resourceData)[0]; // coke_ingestor
    resourceData[dispenserName].icons[dispenserName] = '/dispenser';
    const icon = manager.getIconPath(dispenserName);
    expect(icon).toBeTruthy();
  });
});

describe('Correct functionally of getters', () => {
  test('get all dispensers', () => {
    const resourceData = fakeResources().dispensers as Record<string, DispenserResource>;
    const manager = new DispenserResourceManager(resourceData);
    expect(manager.all).toEqual(fakeResources().dispensers);
  });

  test('Convert dict to an array of dicts', () => {
    const manager = new DispenserResourceManager({
      testDispenser: {
        icons: {
          testDispenser: '',
        },
        location: { x: 1, y: 1, yaw: 1, level_name: 'L1' },
      },
    });
    expect(manager.allValues).toEqual([
      {
        icons: {
          testDispenser: '',
        },
        location: { x: 1, y: 1, yaw: 1, level_name: 'L1' },
        guid: 'testDispenser',
      },
    ]);
  });
});
