import {
  RenderResult,
  act,
  cleanup,
  fireEvent,
  render as render_,
  waitFor,
} from '@testing-library/react';
import React from 'react';
import { RobotMarker } from './robot-marker';
import { makeRobotData } from './test-utils.spec';
import { RobotData } from './three-fiber';

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
      await render(<RobotMarker cx={0} cy={0} r={1} color={robot.color} />);
      cleanup();
    }
  });

  it('trigger onClick event', async () => {
    const onClick = jest.fn();
    const root = await render(
      <RobotMarker cx={0} cy={0} r={1} color="#000000" onClick={onClick} data-testid="marker" />,
    );
    fireEvent.click(root.getByTestId('marker'));

    await waitFor(() => {
      expect(onClick).toHaveBeenCalled();
    });
  });

  it('providing iconPath renders an image', async () => {
    const root = await render(
      <RobotMarker
        cx={0}
        cy={0}
        r={1}
        color="#000000"
        iconPath="/base/test-data/assets/tiny-robot.png"
      />,
    );
    expect(root.container.querySelector('image')).not.toBeNull();
  });
});
