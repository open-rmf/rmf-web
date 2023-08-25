# Settings

There are some environment variables that control how the test runs.

| Key | Description |
|---|---|
| E2E_NO_DASHBOARD | _(bool)_ Do not launch the dashboard server |
| E2E_NO_RMF_SERVER | _(bool)_ Do not launch the rmf api server |
| E2E_DASHBOARD_URL | _(string)_ Base url where the dashboard is hosted |

Boolean values can be 0/1/true/false.

There are also some environment variables the test runner sets by default

| Key | Default Value |
|---|---|
| REACT_APP_TRAJECTORY_SERVER | ws://localhost:8006 |
| E2E_DASHBOARD_URL | http://localhost:5000 |

You can overwrite them by setting them in your environment variables.

# Regarding wdio async Selectors Chaining

**NOTE: It is recommened to NOT use async selectors chaining.**

wdio selectors chaining is a useful way to simplify code by allowing async selectors to be chained as if they are sync selectors, for example

```ts
await appBar.$('span=Tasks').click();
```

However in practice, as of v7.11.1, this is found to be very buggy and will often fail for unexplained reasons. These are some snippets of code which will cause it to fail.

```js
browser.overwriteCommand(
  'click',
  async function (orig, { force = false, ...clickOpts } = {}) {
    if (force) {
      await browser.execute(function click(el) {
        el.dispatchEvent(new MouseEvent('click', { bubbles: true })); // eslint-disable-line no-undef
      }, this);
      await browser.pause(500);
      return;
    }
    // BUG: calling `waitForClickable` seems to cause wdio chain selectors to stop working sometimes.
    await this.waitForClickable();
    await orig.apply(this, [clickOpts]);
    await browser.pause(500);
  },
  true,
);
```

```ts
// this will fail depending on the context on which `getPatrolOption` is called.
const getPatrolOption = async () => {
  return $$('[role=option]').find(async (elem) => await elem.getText() === 'Patrol');
};

await browser.waitUntil(async () => !!(await getPatrolOption)); // ok
await console.log(await getPatrolOption()); // error
```

Possible reason is because async selectors chaining relies on a "finalizer" or some kind of context to resolve the promises. As a result, when running `getPatrolOption` without a wdio await, the chaining does not work. But this is all speculation, the inner workings of async selector chainings are very complex.
