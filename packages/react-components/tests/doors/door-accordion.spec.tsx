import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { render, RenderResult } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { DoorAccordion } from '../../lib';
import { makeDoor } from './test-utils';

describe('Door-accordion', () => {
  let fakeOnClick: ReturnType<typeof jasmine.createSpy>;
  let root: RenderResult;
  beforeEach(() => {
    fakeOnClick = jasmine.createSpy();
    root = render(<DoorAccordion door={makeDoor()} onDoorControlClick={fakeOnClick} />);
  });

  it('triggers door control dispatch when open door button is clicked', () => {
    userEvent.click(root.getByText('Open'));
    expect(fakeOnClick).toHaveBeenCalledWith(
      jasmine.anything(),
      jasmine.anything(),
      RomiCore.DoorMode.MODE_OPEN,
    );
  });

  it('triggers door control dispatch when close door button is clicked', () => {
    userEvent.click(root.getByText('Close'));
    expect(fakeOnClick).toHaveBeenCalledWith(
      jasmine.anything(),
      jasmine.anything(),
      RomiCore.DoorMode.MODE_CLOSED,
    );
  });
});
