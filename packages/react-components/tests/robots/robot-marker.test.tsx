import { act, render as render_, RenderResult } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { ColorContext, ColorManager, RobotMarker } from '../../lib';
import { makeRobot } from './test-utils';

let colorManager: ColorManager;

beforeEach(() => {
  // TextEncoder is not available in node
  colorManager = new ColorManager();
  colorManager.robotPrimaryColor = jest.fn(async () => 'black');
});

async function render(Component: JSX.Element): Promise<RenderResult> {
  const renderImpl = () => {
    return render_(
      <ColorContext.Provider value={colorManager}>
        <svg>{Component}</svg>
      </ColorContext.Provider>,
    );
  };

  let root: ReturnType<typeof renderImpl>;
  await act(async () => {
    root = render_(
      <ColorContext.Provider value={colorManager}>
        <svg>{Component}</svg>
      </ColorContext.Provider>,
    );
  });

  return root!;
}

test('trigger onClick event', async () => {
  const handler = jest.fn();
  const root = await render(
    <RobotMarker
      robot={makeRobot()}
      fleetName="test_fleet"
      footprint={1}
      onClick={handler}
      data-testid="marker"
    />,
  );
  userEvent.click(root.getByTestId('marker'));
  expect(handler).toBeCalled();
});

test('providing iconPath renders an image', async () => {
  const root = await render(
    <RobotMarker robot={makeRobot()} fleetName="test_fleet" footprint={1} iconPath="test_icon" />,
  );
  expect(root.container.querySelector('image')).not.toBeNull();
});

test("uses ColorManager to determine robot's color", async () => {
  await render(<RobotMarker robot={makeRobot()} fleetName="test_fleet" footprint={1} />);
  expect(colorManager.robotPrimaryColor).toHaveBeenCalled();
});
