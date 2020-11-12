import React from 'react';
import { SnapshotControlButtonGroup } from '../lib';
import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';

test('It disabled buttons', () => {
  const root = render(
    <SnapshotControlButtonGroup
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

test('It executes callbacks correctly', () => {
  const onResetClick = jest.fn();
  const onClearClick = jest.fn();
  const onRestoreClick = jest.fn();
  const onSaveClick = jest.fn();

  const root = render(
    <SnapshotControlButtonGroup
      onResetClick={onResetClick}
      onClearClick={onClearClick}
      onRestoreClick={onRestoreClick}
      onSaveClick={onSaveClick}
    />,
  );

  userEvent.click(root.getByText('Clear'));
  userEvent.click(root.getByText('Reset'));
  userEvent.click(root.getByText('Restore'));
  userEvent.click(root.getByText('Save'));

  expect(onResetClick).toHaveBeenCalledTimes(1);
  expect(onClearClick).toHaveBeenCalledTimes(1);
  expect(onRestoreClick).toHaveBeenCalledTimes(1);
  expect(onSaveClick).toHaveBeenCalledTimes(1);
});

test('It hides buttons', () => {
  const root = render(
    <SnapshotControlButtonGroup
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
