import { RmfLauncher } from '../rmf-launcher';
import { overwriteClick } from './utils';

describe('loop request', () => {
  const launcher = new RmfLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  before(() => overwriteClick());
  before(() => browser.url('/'));

  it('rmf responds to loop request', () => {
    browser.addLocatorStrategy('findAllRobots', () => {
      return document.querySelectorAll('[data-component=RobotItem]');
    });
    $('[data-component=MainMenu] [data-item=Robots]').click();
    const allRobotItems = browser.custom$$('findAllRobots', '[data-component=RobotItem]');
    let getRobotLocations = () => {
      let robotLocations = allRobotItems.map(robot => {
        robot.click();
        return robot.getLocation();
      });
      return robotLocations;
    };
    const currentRobotLocations = getRobotLocations();

    const backButton = $('[name="back-button"]');
    backButton.click();
    $('[data-component=MainMenu] [data-item=Commands]').click();
    $('input[name=numLoops]').setValue(1);
    $('button=Request').click();

    backButton.click();
    $('[data-component=MainMenu] [data-item=Robots]').click();
    const newRobotLocations = getRobotLocations();

    expect(newRobotLocations).not.toMatchObject(currentRobotLocations);
  });
});
