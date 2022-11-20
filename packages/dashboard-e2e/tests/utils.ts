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
