import ColorManager from '../colors';

describe('Color manager', () => {
  let colorManager: ColorManager;
  const mockRobot = {
    name: 'robot',
    model: 'A',
    fleetName: 'fleet',
  };

  beforeEach(() => {
    colorManager = new ColorManager();
  });

  it('should store and return a color when robotColor is called', async () => {
    const colorPromise = await colorManager.robotColor(
      mockRobot.fleetName,
      mockRobot.name,
      mockRobot.model,
    );
    expect(colorPromise).toEqual(
      colorManager.robotColorFromCache(mockRobot.fleetName, mockRobot.name),
    );
  });

  it('should store and return a color in pathColorCache when robotPrimary Color is called without image path', async () => {
    const colorPromise = await colorManager.robotPrimaryColor(
      mockRobot.fleetName,
      mockRobot.name,
      mockRobot.model,
    );
    expect(colorPromise).toEqual(
      colorManager.pathColorFromCache(mockRobot.fleetName, mockRobot.name),
    );
  });

  it('should return a promise and store color in robotColorCache when robotPrimaryColor is called with image path', (done) => {
    const mockLink = 'link.com';

    setImmediate(() => {
      const colorPromise = colorManager.robotPrimaryColor(
        mockRobot.fleetName,
        mockRobot.name,
        mockRobot.model,
        mockLink,
      );
      expect(colorPromise).resolves.toEqual(
        colorManager.robotColorFromCache(mockRobot.fleetName, mockRobot.name),
      );
      done();
    });
  });
});
