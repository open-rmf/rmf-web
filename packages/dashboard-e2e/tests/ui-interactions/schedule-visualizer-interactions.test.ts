import { OverwriteClickOptions } from '@wdio/sync';
import { closeOmniPanel, getScheduleVisualizer, omniPanelMainMenu } from '../utils';

describe('schedule visualizer interactions', () => {
  let clickOpts: OverwriteClickOptions;

  beforeEach(() => {
    clickOpts = {
      // `getClientRects` which is needed to wait for element to be clickable
      // does not work properly for svg components in safari.
      force: browser.requestedCapabilities.browserName === 'safari',
    };
  });

  it('clicking a door on the map focuses it on the panel', () => {
    closeOmniPanel();
    const scheduleVisualizer = getScheduleVisualizer();
    const door = scheduleVisualizer.$(`[aria-label=main_door]`);
    door.click(clickOpts);

    expect($('.MuiAccordion-root*=main_door').$('.MuiAccordionDetails-root')).toBeVisible();
  });

  it('clicking a dispenser on the map focuses it on the panel', () => {
    closeOmniPanel();
    const scheduleVisualizer = getScheduleVisualizer();
    const dispenser = scheduleVisualizer.$('[aria-label=coke_dispenser]');
    dispenser.click(clickOpts);

    expect($('.MuiAccordion-root*=coke_dispenser').$('.MuiAccordionDetails-root')).toBeVisible();
  });

  it('clicking a robot on the map focuses it on the panel', () => {
    closeOmniPanel();
    const scheduleVisualizer = getScheduleVisualizer();
    const robot = scheduleVisualizer.$('[aria-label=tinyRobot1]');
    robot.click(clickOpts);

    expect($('.MuiAccordion-root*=tinyRobot1').$('.MuiAccordionDetails-root')).toBeVisible();
  });

  // filter test
  it('Clicking on an equipment on the map resets the filter', () => {
    omniPanelMainMenu();
    $('[data-component=MainMenu] [data-item=Doors]').click();
    // set value to filter input
    $('[data-component=simple-filter]').$('input').setValue('value');

    // get door marker and click
    const scheduleVisualizer = getScheduleVisualizer();
    const door = scheduleVisualizer.$('[aria-label=main_door]');

    door.click(clickOpts);

    // check that the filter is empty after clicking
    expect($('[data-component=simple-filter]').$('input').getValue()).toEqual('');
    // check that door panel is expanded
    expect($('.MuiAccordion-root*=main_door').$('.MuiAccordionDetails-root')).toBeVisible();
  });
});
