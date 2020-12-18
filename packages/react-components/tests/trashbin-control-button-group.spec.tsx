import React from 'react';
import { TrashBinControlButtonGroup } from '../lib';
import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';

it('It disabled buttons', () => {
  const root = render(
    <TrashBinControlButtonGroup
      disableClear={true}
      disableReset={true}
      disableRestore={true}
      disableSave={true}
    />,
  );
  expect(
    root.container.querySelector('button#clear-button')?.hasAttribute('disabled'),
  ).toBeTruthy();
  expect(
    root.container.querySelector('button#reset-button')?.hasAttribute('disabled'),
  ).toBeTruthy();
  expect(
    root.container.querySelector('button#restore-button')?.hasAttribute('disabled'),
  ).toBeTruthy();
  expect(root.container.querySelector('button#save-button')?.hasAttribute('disabled')).toBeTruthy();
});

it('It executes callbacks correctly', () => {
  const handler = {
    onResetClick: () => {},
    onClearClick: () => {},
    onRestoreClick: () => {},
    onSaveClick: () => {},
  };

  spyOn(handler, 'onResetClick');
  spyOn(handler, 'onClearClick');
  spyOn(handler, 'onRestoreClick');
  spyOn(handler, 'onSaveClick');

  const root = render(
    <TrashBinControlButtonGroup
      onResetClick={handler.onResetClick}
      onClearClick={handler.onClearClick}
      onRestoreClick={handler.onRestoreClick}
      onSaveClick={handler.onSaveClick}
    />,
  );

  userEvent.click(root.getByText('Clear'));
  userEvent.click(root.getByText('Reset'));
  userEvent.click(root.getByText('Restore'));
  userEvent.click(root.getByText('Save'));

  expect(handler.onResetClick).toHaveBeenCalledTimes(1);
  expect(handler.onClearClick).toHaveBeenCalledTimes(1);
  expect(handler.onRestoreClick).toHaveBeenCalledTimes(1);
  expect(handler.onSaveClick).toHaveBeenCalledTimes(1);
});

it('It hides buttons', () => {
  const root = render(
    <TrashBinControlButtonGroup
      showReset={false}
      showRestore={false}
      showSave={false}
      showClear={false}
    />,
  );
  expect(root.container.querySelector('button#clear-button')).toBeFalsy();
  expect(root.container.querySelector('button#reset-button')).toBeFalsy();
  expect(root.container.querySelector('button#restore-button')).toBeFalsy();
  expect(root.container.querySelector('button#save-button')).toBeFalsy();
});
