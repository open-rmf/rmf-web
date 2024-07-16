import { getAppBar } from '../utils';

describe('submit task', () => {
  it('can submit patrol task', async () => {
    const appBar = await getAppBar();
    await (await appBar.$('button[aria-label="Tasks"]')).click();
    await (await appBar.$('button[aria-label="new task"]')).click();
    await (await $('#task-type')).click();
    const getPatrolOption = async () => {
      const options = await $$('[role=option]');
      for (const opt of options) {
        const text = await opt.getText();
        if (text === 'Patrol') {
          return opt;
        }
      }
      return null;
    };
    await browser.waitUntil(async () => !!(await getPatrolOption()));
    const patrolOption = (await getPatrolOption())!;
    await patrolOption.click();

    await (await $('#place-input')).click();
    const getCoeOption = async () => {
      const options = await $$('[role=option]');
      for (const opt of options) {
        const text = await opt.getText();
        if (text === 'coe') {
          return opt;
        }
      }
      return null;
    };
    await browser.waitUntil(async () => !!(await getCoeOption()));
    const coeOption = (await getCoeOption())!;
    await coeOption.click();

    await (await $('button[aria-label="Submit Now"]')).click();
    await expect($('div=Successfully created task')).toBeDisplayed();
  }).timeout(60000);

  it('can submit favorite task', async () => {
    const appBar = await getAppBar();
    await (await appBar.$('button[aria-label="Tasks"]')).click();
    await (await appBar.$('button[aria-label="new task"]')).click();
    await (await $('#task-type')).click();
    const getPatrolOption = async () => {
      const options = await $$('[role=option]');
      for (const opt of options) {
        const text = await opt.getText();
        if (text === 'Patrol') {
          return opt;
        }
      }
      return null;
    };
    await browser.waitUntil(async () => !!(await getPatrolOption()));
    const patrolOption = (await getPatrolOption())!;
    await patrolOption.click();

    await (await $('#place-input')).click();
    const getCoeOption = async () => {
      const options = await $$('[role=option]');
      for (const opt of options) {
        const text = await opt.getText();
        if (text === 'coe') {
          return opt;
        }
      }
      return null;
    };
    await browser.waitUntil(async () => !!(await getCoeOption()));
    const coeOption = (await getCoeOption())!;
    await coeOption.click();

    await (await $('button[aria-label="Save as a favorite task"]')).click();
    await (await $('#favorite-input')).waitForDisplayed();
    await (await $('#favorite-input')).setValue('My favorite task');
    await (await $('button[aria-label="Confirm"]')).click();

    await expect($('div=Created favorite task successfully')).toBeDisplayed();

    // Check if it controls that the name of the favorite task cannot be duplicated correctly.
    await (await $('button[aria-label="Save as a favorite task"]')).click();
    await (await $('#favorite-input')).waitForDisplayed();
    await (await $('#favorite-input')).setValue('My favorite task');
    await (await $('button[aria-label="Confirm"]')).click();
    await expect(
      $(
        'div=Failed to create or delete favorite task: The name of the favorite task already exists.',
      ),
    ).toBeDisplayed();

    await (await $('#favorite-input')).waitForDisplayed();
    // Concatenates the name of the previous task and adds "two" to it.
    await (await $('#favorite-input')).setValue(' two');
    await (await $('button[aria-label="Confirm"]')).click();
    await expect($('div=Created favorite task successfully')).toBeDisplayed();

    const favoriteTaskItem = await $(`div=${'My favorite task'}`);
    await favoriteTaskItem.click();

    const deleteButton = await $('button[aria-label="delete"]');
    await deleteButton.click();

    const confirmButton = await $('button[aria-label="Confirm"]');
    await confirmButton.click();

    await (await $('button[aria-label="Cancel button"]')).click();
  }).timeout(300000);
});
