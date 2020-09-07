import ColorManager from '../colors';

describe('Color manager', () => {
  let colorManager: ColorManager;
  const mockRobot = {
    name: 'robot',
    model: 'A',
  };

  beforeEach(() => {
    colorManager = new ColorManager();
  });

  it('returns a color when robotColor is called', async () => {
    const colorPromise = await colorManager.robotColor(mockRobot.name, mockRobot.model);

    expect(colorPromise).toEqual(colorManager.robotColorFromCache(mockRobot.name));
  });
});
