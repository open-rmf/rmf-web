import fetch from 'node-fetch';

/**
 * Overwrites the default click command to wait for animation to finish before attempting to click,
 * this can fix flaky tests where the click is missed as the position changes as the animation is
 * running.
 */
export function overwriteClick(): void {
  browser.overwriteCommand(
    'click',
    async function (this: WebdriverIO.Element, origClick) {
      await this.waitForClickable();
      let prevLocation = await this.getLocation();
      await this.waitUntil(async () => {
        const newLocation = await this.getLocation();
        const stablized = prevLocation.x === newLocation.x && prevLocation.y === newLocation.y;
        prevLocation = newLocation;
        return stablized;
      });
      return origClick();
    },
    true,
  );
}

/**
 * Return a list of backspace characters. This is only used in case we want to delete characters from the Autocomplete material-ui component
 */
export function removeTextFromAutocomplete(characterNum: number): string {
  const backspace = '\u0008';
  let backspaces = '';
  for (let index = 0; index < characterNum; index++) {
    backspaces += backspace;
  }
  return backspaces;
}

export const populateDatabase = async (
  data: { log: string; stream: string }[],
): Promise<void | Error> => {
  const options = {
    method: 'POST',
    body: JSON.stringify(data),
    headers: {
      'Content-Type': 'application/json',
    },
  };

  try {
    const res = await fetch('http:localhost:8003/log/rmfserver', options);
    return await res.json();
  } catch (error) {
    return error as Error;
  }
};

export const selectDateAndRetrieveLogs = async (): Promise<void> => {
  const datePickerIconButton = await (await $('.MuiInputBase-root')).$('.MuiIconButton-root');
  await datePickerIconButton.click();
  const prevMonthButton = (await $$('.MuiPickersCalendarHeader-iconButton'))[0];
  await prevMonthButton.click();
  const dayOneButton = await $('button=1');
  await dayOneButton.click();
  await (await $('body')).click();
  await (await $('button=Retrieve Logs')).click();
  await browser.pause(1000);
};

export const getReport = async (options: {
  listOrder: number;
  elemName: string;
  reportTitle: string;
}): Promise<void> => {
  await browser.waitUntil(
    async () => (await (await $('.MuiList-root')).waitForDisplayed()) === true,
  );
  const targetButton = await (
    await $(`.MuiList-root .MuiListItem-root:nth-child(${options.listOrder})`)
  ).$(options.elemName);
  await targetButton.click();
  await selectDateAndRetrieveLogs();
  await browser.waitUntil(
    async () => (await (await $(options.reportTitle)).waitForDisplayed({ timeout: 5000 })) === true,
  );
};
