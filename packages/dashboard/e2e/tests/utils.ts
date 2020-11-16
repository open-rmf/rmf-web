import { Element } from '@wdio/sync';

/**
 * Overwrites the default click command to wait for animation to finish before attempting to click,
 * this can fix flaky tests where the click is missed as the position changes as the animation is
 * running.
 */
export function overwriteClick() {
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
  const allRobotItems = browser.custom$$('findAllRobots', '[data-component=RobotItem]');
  let robotLocations = allRobotItems.map((robot) => {
    robot.click();
    return robot.$('[data-role=position]').getHTML();
  });
  return robotLocations;
};

export function login(): void {
  browser.url('/login');
  $('#login-button').click();
  $('#username').setValue('admin');
  $('#password').setValue('admin');
  $('#kc-login').click();
}

/**
 * Open loop form
 * Obs: Make sure that the OmniPanel is visible and in the root to run this.
 */
export const openRequestForm = () => {
  $('[data-component=MainMenu] [data-item=Commands]').click();

  const loopForm = $('[data-component=LoopForm]');
  loopForm.click();
};

interface E2eRequestLoopProps {
  pointA: string;
  pointB: string;
  loopNumber?: number;
  charsToRemoveFromPointA?: number;
  charsToRemoveFromPointB?: number;
}

/**
 * Request a loop task to the robots.
 * Obs: Make sure that the OmniPanel is visible and it's placed on the loop form.
 */
export const requestLoop = (props: E2eRequestLoopProps): void => {
  const { pointA, pointB, loopNumber, charsToRemoveFromPointA, charsToRemoveFromPointB } = props;
  const loopForm = $('[data-component=LoopForm]');

  $('input[name=startLocation]').waitForClickable();
  $('input[name=startLocation]').setValue(
    removeTextFromAutocomplete(charsToRemoveFromPointA || 10),
  );
  $('input[name=startLocation]').setValue(pointA);
  $('.MuiAutocomplete-popper').click();

  $('input[name=finishLocation]').waitForClickable();
  $('input[name=finishLocation]').setValue(
    removeTextFromAutocomplete(charsToRemoveFromPointB || 20),
  );
  $('input[name=finishLocation]').setValue(pointB);
  $('.MuiAutocomplete-popper').click();

  $('input[name=numLoops]').waitForClickable();
  $('input[name=numLoops]').setValue(loopNumber || 1);
  loopForm.$('button=Request').click();
};
