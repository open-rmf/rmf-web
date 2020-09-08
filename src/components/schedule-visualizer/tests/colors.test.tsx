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

  it('should store and return a color when robotColor is called', async () => {
    const colorPromise = await colorManager.robotColor(mockRobot.name, mockRobot.model);
    expect(colorPromise).toEqual(colorManager.robotColorFromCache(mockRobot.name));
  });

  it('should store and return a color when robotTrajectoryColor is called', async () => {
    const colorPromise = await colorManager.robotTrajectoryColor(mockRobot.name, mockRobot.model);
    expect(colorPromise).toEqual(colorManager.pathColorFromCache(mockRobot.name));
  });

  it('should return a promise when robotImageColor is called', done => {
    const mockLink = 'link.com';

    setImmediate(() => {
      const colorPromise = colorManager.robotImageColor(mockLink, mockRobot.name);
      expect(colorPromise).resolves.toEqual(colorManager.robotColorFromCache(mockRobot.name));
      done();
    });
  });
});
