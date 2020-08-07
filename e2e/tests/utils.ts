import { Element } from '@wdio/sync';

/**
 * Overwrites the default click command to wait for animation to finish before attempting to click,
 * this can fix flaky tests where the click is missed as the position changes as the animation is
 * running.
 */
export function overwriteClick() {
  browser.overwriteCommand(
    'click',
    function(this: Element, origClick) {
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
  const allRobotItems = browser.custom$$('findAllRobots', '[data-component=RobotItem]');
  let robotLocations = allRobotItems.map(robot => {
    robot.click();
    return robot.$('[data-role=position]').getHTML();
  });
  return robotLocations;
};
