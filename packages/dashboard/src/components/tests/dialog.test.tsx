import React from 'react';
import { DialogTitle } from '../dialog';
import { createMount } from '@material-ui/core/test-utils';
import { IconButton } from '@material-ui/core';

const mount = createMount();

describe('Test custom Dialog elements', () => {
  test('Dialog title has title and closeIcon', () => {
    const root = mount(
      <DialogTitle id="test" onClose={() => jest.fn()}>
        Hotkeys
      </DialogTitle>,
    );
    expect(root.find('h6').html()).toContain('Hotkeys');
    expect(root.find(IconButton)).toBeTruthy();
  });
});
