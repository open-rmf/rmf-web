import { render, RenderResult } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { DoorCell } from './door-cell';
import { makeDoor } from './test-utils.spec';

describe('Door Panel', () => {
  const door = { door: makeDoor({ name: 'test_door' }), level: 'test_level' };
  const doorMode = RmfModels.DoorMode.MODE_OPEN;
  let root: RenderResult;
  let mockControlClickSubmit: jasmine.Spy<jasmine.Func>;

  beforeEach(() => {
    mockControlClickSubmit = jasmine.createSpy();
    root = render(
      <DoorCell door={door} doorMode={doorMode} onDoorControlClick={mockControlClickSubmit} />,
    );
  });

  it('should call onDoorControlClick when Open/Close button is clicked', () => {
    userEvent.click(root.getByRole('button', { name: 'Open' }));
    expect(mockControlClickSubmit).toHaveBeenCalled();
  });
});
