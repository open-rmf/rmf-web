import { Element } from '@wdio/sync';

/**
 * Overwrites the default click command to wait for animation to finish before attempting to click,
 * this can fix flaky tests where the click is missed as the position changes as the animation is
 * running.
 */
export function overwriteClick(): void {
  browser.overwriteCommand(
    'click',
    function (this: Element, origClick) {
      let prevLocation = this.getLocation();
      this.waitUntil(() => {
        const newLocation = this.getLocation();
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

/**
 * Get the robot location
 */
export const getRobotLocations = (browser: WebdriverIO.BrowserObject): string[] => {
  const allRobotItems = browser.$$('[data-component=RobotAccordion]');
  const robotLocations = allRobotItems.map((robot) => {
    robot.click();
    const getLocations = () => {
      const items = robot.$$('[role=row]');
      items[items.length - 1].scrollIntoView();
      return items.filter((el) => el.getText().startsWith('Location'));
    };
    browser.waitUntil(() => getLocations().length > 0);
    const location = getLocations()[0];
    return location.getText();
  });
  return robotLocations;
};

export function getAppBar(): Element {
  return $('#appbar');
}

export function getScheduleVisualizer(): Element {
  return $('#schedule-visualizer');
}

export function getOmniPanel(): Element {
  return $('#omnipanel');
}

export function closeOmniPanel(): void {
  $(`#omnipanel [aria-label=Close]`).click();
}

export function openOmniPanel(): void {
  $('#omnipanel-control').click();
}

export function omniPanelMainMenu(): void {
  $(`#omnipanel [aria-label=Home]`).click();
}

export function getDoorAccordion(doorName: string): Element {
  return $(`.MuiAccordion-root*=${doorName}`);
}
