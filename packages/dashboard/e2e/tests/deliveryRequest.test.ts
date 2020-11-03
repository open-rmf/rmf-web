import { makeLauncher } from '../rmf-launcher';
import { getRobotLocations, login, overwriteClick, removeTextFromAutocomplete } from './utils';

describe('Delivery request', () => {
  const launcher = makeLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  before(() => overwriteClick());
  before(() => browser.url('/'));

  before(login);

  it('rmf responds to delivery request', () => {
    $('[data-component=MainMenu] [data-item=Robots]').click();

    const currentRobotLocations = getRobotLocations(browser);

    const backButton = $('[aria-label="Back"]');
    backButton.click();
    $('[data-component=MainMenu] [data-item=Commands]').click();
    const deliveryForm = $('[data-component=DeliveryForm]');
    deliveryForm.click();
    $('input[name=pickupPlace]').waitForClickable();
    $('input[name=pickupPlace]').setValue(removeTextFromAutocomplete(20));
    $('input[name=pickupPlace]').waitForClickable();
    $('input[name=pickupPlace]').setValue('pantry');
    $('.MuiAutocomplete-popper').click();

    $('input[name=pickupDispenser]').click();
    $('.MuiAutocomplete-popper').click();

    $('input[name=dropoffPlace]').waitForClickable();
    $('input[name=dropoffPlace]').setValue(removeTextFromAutocomplete(20));
    $('input[name=dropoffPlace]').waitForClickable();
    $('input[name=dropoffPlace]').setValue('hardware_2');
    $('.MuiAutocomplete-popper').click();

    $('input[name=dropoffDispenser]').click();
    $('.MuiAutocomplete-popper').click();

    const requestButton = deliveryForm.$('button=Request');
    requestButton.scrollIntoView();
    requestButton.click();

    backButton.click();
    $('[data-component=MainMenu] [data-item=Robots]').click();
    const newRobotLocations = getRobotLocations(browser);

    expect(newRobotLocations).not.toMatchObject(currentRobotLocations);
  });

  it('renders robot trajectory', () => {
    expect($('[data-component=RobotTrajectory]')).toBeVisible();
  });
});
