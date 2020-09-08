import { RmfLauncher } from '../rmf-launcher';
import { getRobotLocations, login, overwriteClick, removeTextFromAutocomplete } from './utils';

describe('Delivery request', () => {
  const launcher = RmfLauncher.instance;

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  before(() => overwriteClick());
  before(() => browser.url('/'));

  before(login);

  it('rmf responds to delivery request', () => {
    $('[data-component=MainMenu] [data-item=Robots]').click();

    browser.addLocatorStrategy('findAllRobots', () => {
      return document.querySelectorAll('[data-component=RobotItem]');
    });

    const currentRobotLocations = getRobotLocations(browser);

    const backButton = $('[name="back-button"]');
    backButton.click();
    $('[data-component=MainMenu] [data-item=Commands]').click();
    const deliveryForm = $('[data-component=DeliveryForm]');
    deliveryForm.click();
    $('input[name=pickupPlace]').waitForClickable();
    $('input[name=pickupPlace]').setValue(removeTextFromAutocomplete(10));
    $('input[name=pickupPlace]').setValue('pantry');
    $('.MuiAutocomplete-popper').click();

    $('input[name=pickupDispenser]').click();
    $('.MuiAutocomplete-popper').click();

    $('input[name=dropoffPlace]').setValue(removeTextFromAutocomplete(20));
    $('input[name=dropoffPlace]').setValue('hardware_2');
    $('.MuiAutocomplete-popper').click();

    $('input[name=dropoffDispenser]').click();
    $('.MuiAutocomplete-popper').click();

    deliveryForm.$('button=Request').click();

    backButton.click();
    $('[data-component=MainMenu] [data-item=Robots]').click();
    const newRobotLocations = getRobotLocations(browser);

    expect(newRobotLocations).not.toMatchObject(currentRobotLocations);
  });

  it('renders robot trajectory', () => {
    expect($('[data-component=RobotTrajectory]')).toBeVisible();
  });
});
