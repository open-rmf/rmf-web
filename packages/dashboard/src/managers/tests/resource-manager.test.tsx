import ResourceManager from '../resource-manager';
import fakeResources from '../__mocks__/resources';

describe('Correct instantiation', () => {
  test('Instantiate robots and dispensers', () => {
    const resources = new ResourceManager(fakeResources());
    expect(!!resources.robots).toBe(true);
    expect(!!resources.dispensers).toBe(true);
  });

  test(`Instantiate just robots`, () => {
    const rawResources = { robots: fakeResources().robots, dispensers: undefined };
    const resources = new ResourceManager(rawResources);
    expect(!!resources.robots).toBe(true);
    expect(resources.dispensers).toBe(undefined);
  });
});
