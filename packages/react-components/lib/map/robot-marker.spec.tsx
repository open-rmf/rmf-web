import { act, cleanup, render as render_, RenderResult } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { makeRobot } from '../robots/test-utils.spec';
import { RobotMarker } from './robot-marker';
import { RobotData } from './robots-overlay';
import { makeRobotData } from './test-utils.spec';

describe('RobotMarker', () => {
  async function render(Component: JSX.Element): Promise<RenderResult> {
    let root: RenderResult;
    await act(async () => {
      root = render_(<svg>{Component}</svg>);
    });

    return root!;
  }

  it('smoke test', async () => {
    const robots: RobotData[] = [
      makeRobotData({
        name: 'test_robot_1',
        inConflict: false,
      }),
      makeRobotData({
        name: 'test_robot_2',
        inConflict: true,
      }),
    ];
    for (const robot of robots) {
      await render(
        <RobotMarker
          fleet={robot.fleet}
          name={robot.name}
          model={robot.model}
          footprint={robot.footprint}
          color="#000000"
          state={makeRobot({ name: robot.name })}
        />,
      );
      cleanup();
    }
  });

  it('trigger onClick event', async () => {
    const robot = makeRobotData();
    const onClick = jasmine.createSpy();
    const root = await render(
      <RobotMarker
        fleet={robot.fleet}
        name={robot.name}
        model={robot.model}
        footprint={robot.footprint}
        color="#000000"
        state={makeRobot({ name: robot.name })}
        onClick={onClick}
        data-testid="marker"
      />,
    );
    userEvent.click(root.getByTestId('marker'));
    expect(onClick).toHaveBeenCalled();
  });

  it('providing iconPath renders an image', async () => {
    const robot = makeRobotData();
    const root = await render(
      <RobotMarker
        fleet={robot.fleet}
        name={robot.name}
        model={robot.model}
        footprint={robot.footprint}
        color="#000000"
        state={makeRobot({ name: robot.name })}
        iconPath="/base/test-data/assets/tiny-robot.png"
      />,
    );
    expect(root.container.querySelector('image')).not.toBeNull();
  });
});
