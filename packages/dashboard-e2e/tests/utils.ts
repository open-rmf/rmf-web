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

export function getAppBar(): ReturnType<WebdriverIO.Browser['$']> {
  return $('#appbar');
}

export function getScheduleVisualizer(): ReturnType<WebdriverIO.Browser['$']> {
  return $('#schedule-visualizer');
}

export function getOmniPanel(): ReturnType<WebdriverIO.Browser['$']> {
  return $('#omnipanel');
}

export async function closeOmniPanel(): Promise<void> {
  return (await $(`#omnipanel [aria-label=Close]`)).click();
}

export async function openOmniPanel(): Promise<void> {
  return (await $('#omnipanel-control')).click();
}

export async function omniPanelMainMenu(): Promise<void> {
  return (await $(`#omnipanel [aria-label=Home]`)).click();
}

export function getDoorAccordion(doorName: string): ReturnType<WebdriverIO.Browser['$']> {
  return $(`.MuiAccordion-root*=${doorName}`);
}
