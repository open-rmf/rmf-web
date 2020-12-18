import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { DoorAccordion } from '../../lib';
import { makeDoor } from './test-utils';

describe('Door-accordion', () => {
  let handler: { onClick: () => void };
  let root: any;
  beforeEach(() => {
    handler = { onClick: () => {} };
    spyOn(handler, 'onClick');
    root = render(<DoorAccordion door={makeDoor()} onDoorControlClick={handler.onClick} />);
  });

  it('triggers door control dispatch when open door button is clicked', () => {
    userEvent.click(root.getByText('Open'));
    expect(handler.onClick).toHaveBeenCalledWith(
      jasmine.anything(),
      jasmine.anything(),
      RomiCore.DoorMode.MODE_OPEN,
    );
  });

  it('triggers door control dispatch when close door button is clicked', () => {
    userEvent.click(root.getByText('Close'));
    expect(handler.onClick).toHaveBeenCalledWith(
      jasmine.anything(),
      jasmine.anything(),
      RomiCore.DoorMode.MODE_CLOSED,
    );
  });
});
