import { LogoResource, LogoResourceManager } from '../resource-manager-logos';
import fakeResources from '../__mocks__/resources';

describe('The correct operation of the getIconPath method of the LogoResourceManager class', () => {
  let manager: LogoResourceManager;
  let resourceData: Record<string, LogoResource>;

  beforeAll(() => {
    resourceData = fakeResources().logos as Record<string, LogoResource>;
    manager = new LogoResourceManager(resourceData);
  });

  test('Returns falsy when logoName does not exist', async () => {
    const icon = await manager.getIconPath('Not exists');
    expect(icon).toBeFalsy();
  });

  test('Returns falsy if the icon path it`s empty', async () => {
    resourceData.testLogo = {
      icons: {
        testLogo: '',
      },
    };
    const icon = await manager.getIconPath('testLogo');
    expect(icon).toBeFalsy();
  });

  test('Returns trusty if the icon path exists', () => {
    const logoName = Object.keys(resourceData)[0]; // coke_ingestor
    resourceData[logoName].icons[logoName] = '/logo';
    const icon = manager.getIconPath(logoName);
    expect(icon).toBeTruthy();
  });
});

describe('The correct operation of the getters of the LogoResourceManager class', () => {
  test('Gets all logos with the `all` getter', () => {
    const resourceData = fakeResources().logos as Record<string, LogoResource>;
    const manager = new LogoResourceManager(resourceData);
    const logos = fakeResources().logos;
    const keysOfLogos = logos && Object.keys(logos);
    expect(Object.keys(manager.all)).toEqual(keysOfLogos);
  });
});
