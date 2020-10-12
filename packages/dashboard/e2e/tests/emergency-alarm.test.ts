import { makeLauncher } from '../rmf-launcher';
import {
  getRobotLocations,
  login,
  overwriteClick,
  removeTextFromAutocomplete,
  requestLoop,
} from './utils';

describe('Emergency Alarm', () => {
  const launcher = makeLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  before(() => overwriteClick());
  before(() => browser.url('/'));

  before(login);

  const openAlarmModal = () => {
    const alarmButton = $('[id="alarm-btn"]');
    alarmButton.click();
  };
  const confirmAction = () => {
    const confirmButton = $('.swal2-confirm');
    confirmButton.click();
  };

  it('When the alarm is activated robots go and stay on holding points, when the alarm is deactivated the robot move again ', () => {
    // save robot values
    $('[data-component=MainMenu] [data-item=Robots]').click();
    browser.addLocatorStrategy('findAllRobots', () => {
      return document.querySelectorAll('[data-component=RobotItem]');
    });
    const currentRobotLocations = getRobotLocations(browser);
    const backButton = $('[name="back-button"]');
    backButton.click();

    requestLoop('pantry', 'cubicle_1');
    // wait for the robot to move
    browser.pause(5000);

    // activate Alarm
    openAlarmModal();
    confirmAction();

    // time for the robot to go to a holding point
    browser.pause(10000);

    // check robot position after alarm activation
    backButton.waitForClickable();
    backButton.click();
    $('[data-component=MainMenu] [data-item=Robots]').click();
    const newRobotLocations = getRobotLocations(browser);

    // check that the robot actually moved
    expect(newRobotLocations).not.toMatchObject(currentRobotLocations);

    // The robot should be on a holding point so it should not change its position
    browser.pause(5000);
    const robotLocationAfterActivatedAlarm = getRobotLocations(browser);
    expect(newRobotLocations).toMatchObject(robotLocationAfterActivatedAlarm);

    // Deactivate Alarm
    openAlarmModal();
    confirmAction();

    // Wait for the robot to move again
    browser.pause(5000);
    const robotLocationAfterDeactivateAlarm = getRobotLocations(browser);
    expect(robotLocationAfterActivatedAlarm).not.toMatchObject(robotLocationAfterDeactivateAlarm);
  });
});
