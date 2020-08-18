import { RmfLauncher } from '../rmf-launcher';
import { getRobotLocations, login, overwriteClick } from './utils';

describe('loop request', () => {
  const launcher = new RmfLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  before(() => overwriteClick());
  before(() => browser.url('/'));

  before(login);

  it('rmf responds to loop request', () => {
    $('[data-component=MainMenu] [data-item=Robots]').click();

    browser.addLocatorStrategy('findAllRobots', () => {
      return document.querySelectorAll('[data-component=RobotItem]');
    });

    const currentRobotLocations = getRobotLocations(browser);

    const backButton = $('[name="back-button"]');
    backButton.click();
    $('[data-component=MainMenu] [data-item=Commands]').click();
    $('[data-component=LoopForm]').click();
    $('input[name=numLoops]').waitForClickable();
    $('input[name=numLoops]').setValue(1);
    $('button=Request').click();

    backButton.click();
    $('[data-component=MainMenu] [data-item=Robots]').click();
    const newRobotLocations = getRobotLocations(browser);

    expect(newRobotLocations).not.toMatchObject(currentRobotLocations);
  });
});
