import { Capabilities } from '@wdio/types';
import { closeOmniPanel, getScheduleVisualizer, omniPanelMainMenu } from '../utils';
import { WebdriverIO } from '../wdio';

const getFilterInput = async () => {
  const filter = await $('[data-component=simple-filter]');
  return await filter.$('input');
};

describe('schedule visualizer interactions', () => {
  let clickOpts: WebdriverIO.OverwriteClickOptions;

  beforeEach(() => {
    clickOpts = {
      // `getClientRects` which is needed to wait for element to be clickable
      // does not work properly for svg components in safari.
      force: (browser.requestedCapabilities as Capabilities.Capabilities).browserName === 'safari',
    };
  });

  it('clicking a door on the map focuses it on the panel', async () => {
    await closeOmniPanel();
    const scheduleVisualizer = await getScheduleVisualizer();
    const door = await scheduleVisualizer.$(`[aria-label=main_door]`);
    await door.click(clickOpts);

    await expect(
      (await $('.MuiAccordion-root*=main_door')).$('.MuiAccordionDetails-root'),
    ).toBeDisplayed();
  });

  it('clicking a dispenser on the map focuses it on the panel', async () => {
    await closeOmniPanel();
    const scheduleVisualizer = await getScheduleVisualizer();
    const dispenser = await scheduleVisualizer.$('[aria-label=coke_dispenser]');
    await dispenser.click(clickOpts);

    await expect(
      (await $('.MuiAccordion-root*=coke_dispenser')).$('.MuiAccordionDetails-root'),
    ).toBeDisplayed();
  });

  it('clicking a robot on the map focuses it on the panel', async () => {
    await closeOmniPanel();
    const scheduleVisualizer = await getScheduleVisualizer();
    const robot = await scheduleVisualizer.$('[aria-label=tinyRobot1]');
    await robot.click(clickOpts);

    await expect(
      (await $('.MuiAccordion-root*=tinyRobot1')).$('.MuiAccordionDetails-root'),
    ).toBeDisplayed();
  });

  it('Clicking on an equipment on the map resets the filter', async () => {
    await omniPanelMainMenu();
    await (await $('[data-component=MainMenu] [data-item=Doors]')).click();

    // set value to filter input
    await (await getFilterInput()).setValue('value');

    // get door marker and click
    const scheduleVisualizer = await getScheduleVisualizer();
    const door = await scheduleVisualizer.$('[aria-label=main_door]');

    await door.click(clickOpts);

    // check that the filter is empty after clicking
    expect(await (await getFilterInput()).getValue()).toEqual('');
    // check that door panel is expanded
    await expect(
      (await $('.MuiAccordion-root*=main_door')).$('.MuiAccordionDetails-root'),
    ).toBeDisplayed();
  });
});
