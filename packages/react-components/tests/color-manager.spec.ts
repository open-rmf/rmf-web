import { ColorManager } from '../lib';

let colorManager: ColorManager;

beforeEach(() => {
  colorManager = new ColorManager();
});

it('snapshot - get color based on image', async () => {
  const color = await colorManager.robotPrimaryColor(
    'test_fleet',
    'test_robot',
    'test_model',
    '/base/tests/assets/tiny-robot.png',
  );
  expect(color).toBe('rgb(4, 52, 156)');
});

it('able to get color from cache after the color is loaded previously', async () => {
  const color = await colorManager.robotPrimaryColor('test_fleet', 'test_robot', 'test_model');
  const colorCache = colorManager.robotColorFromCache('test_fleet', 'test_robot');
  expect(colorCache).toBe(color);
});

it('fallback to color from id if image analysis fails', async () => {
  expect(async () => {
    await colorManager.robotPrimaryColor(
      'test_fleet',
      'test_robot',
      'test_model',
      '/non-existing-image.png',
    );
  }).not.toThrow();
});
