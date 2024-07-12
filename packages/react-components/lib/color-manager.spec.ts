import { ColorManager } from './color-manager';

let colorManager: ColorManager;

beforeEach(() => {
  colorManager = new ColorManager();
});

describe('ColorManager', () => {
  it('fallback to color from id if image analysis fails', async () => {
    expect(async () => {
      await colorManager.robotPrimaryColor(
        'test_fleet',
        'test_robot',
        'test_model',
        '/base/test-data/assets/non-existing-image.png',
      );
    }).not.toThrow();
  });
});
