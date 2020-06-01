import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import { UpArrow, DownArrow } from '../arrow';
import { Arrow } from '../arrow';

const mount = createMount();

it('Check correct positive calculation from DownArrow', async () => {
  const wrapper = mount(
    <svg>
      <DownArrow x={2} y={2} />
    </svg>,
  );
  const { x1, y1, x2, y2, x3, y3 } = wrapper
    .find(Arrow)
    .at(0)
    .props();
  expect([x1, y1, x2, y2, x3, y3]).toEqual([2, 2, 1, 0, 3, 0]);
  wrapper.unmount();
});

it('Check correct negative calculation from DownArrow', async () => {
  const wrapper = mount(
    <svg>
      <DownArrow x={2} y={-2} />
    </svg>,
  );
  const { x1, y1, x2, y2, x3, y3 } = wrapper
    .find(Arrow)
    .at(0)
    .props();
  expect([x1, y1, x2, y2, x3, y3]).toEqual([2, -2, 1, -4, 3, -4]);
  wrapper.unmount();
});

it('Check correct negative calculation from UpArrow', async () => {
  const wrapper = mount(
    <svg>
      <UpArrow x={2} y={-2} />
    </svg>,
  );
  const { x1, y1, x2, y2, x3, y3 } = wrapper
    .find(Arrow)
    .at(0)
    .props();
  expect([x1, y1, x2, y2, x3, y3]).toEqual([2, -2, 1, 0, 3, 0]);
  wrapper.unmount();
});
